/**
 * @file
 * @author Edward A. Lee
 *
 * @section LICENSE
Copyright (c) 2020, The University of California at Berkeley and TU Dresden

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * @section DESCRIPTION
 * 
 * Audio functions for MacOS.
 * 
 * See audio_loop_mac.h for instructions.
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include "audio_loop_mac.h"
#include <unistd.h>
#include "AudioToolbox/AudioToolbox.h"

pthread_mutex_t lf_audio_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t lf_audio_cond = PTHREAD_COND_INITIALIZER;

// Pointer to the buffer into which to currently write.
// This is null before the buffer is ready.
int16_t* next_buffer = NULL;
instant_t next_buffer_start_time = NEVER;

struct note {
    lf_waveform_t* waveform;
    int position;   // Starts at 0 when note starts.
    double volume;  // 0.0 for not active.
};

// Array keeping track of notes being played.
struct note notes[NUM_NOTES] = { 0 };

// Notes are added sequentially.
// When we reach the end of the notes array, we cycle
// back to the beginning. If the oldest note has not
// yet finished playing, it will be replaced by the new note.
int note_counter = 0;

/**
 * Add the given value to the current write buffer at the specified index.
 * If the resulting value is larger than what can be represented in
 * the 16-bit short, truncate it.
 * @param index Where in the buffer to add the amplitude.
 * @param value The amplitude to add to whatever amplitude is already there.
 */
void add_to_sound(int index_offset, double value) {
    int sample_value = next_buffer[index_offset] + value;
    if (sample_value > MAX_AMPLITUDE) {
        sample_value = MAX_AMPLITUDE;
    } else if (sample_value < -MAX_AMPLITUDE) {
        sample_value = -MAX_AMPLITUDE;
    }
    next_buffer[index_offset] = (int16_t)sample_value;
}

/**
 * Function that is called by the audio loop to fill the audio buffer
 * with the next batch of audio data.  When this callback occurs,
 * this grabs the mutex lock, copies the buffer that the main program 
 * has been filling into the destination buffer, clears the next
 * buffer, and updates the start time of the next buffer.
 */
void callback (void *ignored, AudioQueueRef queue, AudioQueueBufferRef buf_ref) {
    // Get a C pointer from the reference passed in.
    AudioQueueBuffer *buf = buf_ref;
    
    // Array of samples in the buffer.
    int16_t *samples = buf->mAudioData;
    
    pthread_mutex_lock(&lf_audio_mutex);
    // Make this the new buffer to write into.
    next_buffer = buf->mAudioData;
    // Clear out the next buffer.
    memset(next_buffer, 0, AUDIO_BUFFER_SIZE * sizeof(int16_t));
    next_buffer_start_time += BUFFER_DURATION_NS;
    
    // Fill the buffer with any trailing sample data that
    // didn't fit in the previous buffer.
    for (int note_to_use = 0; note_to_use < NUM_NOTES; note_to_use++) {
        struct note* note_instance = &(notes[note_to_use]);
    
        // Add as much of the note instance into the buffer as will fit.
        for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
            if (note_instance->waveform == NULL || note_instance->volume == 0.0) {
                continue;
            }
            // Calculate the value to add to the sound by averaging all the channels.
            int value = 0;
            for (int channel = 0; channel < note_instance->waveform->num_channels; channel++) {
                value += note_instance->waveform->waveform[note_instance->position + channel];
            }
            value = value / note_instance->waveform->num_channels;
            add_to_sound(i, value * note_instance->volume);
        
            note_instance->position += note_instance->waveform->num_channels;
            if (note_instance->position >= note_instance->waveform->length - note_instance->waveform->num_channels) {
                // Reached the end of the note. Reset the note.
                note_instance->volume = 0.0;
                note_instance->position = 0;
                note_instance->waveform = NULL;
                break;
            }
        }
    }
    
    // Reinsert this same audio buffer at the end of the queue.
    AudioQueueEnqueueBuffer (queue, buf_ref, 0, NULL);
    
    // In case the other thread is waiting for this event, notify
    // (the other thread should not be waiting).
    pthread_cond_signal(&lf_audio_cond);
    pthread_mutex_unlock(&lf_audio_mutex);
}

/**
 * Run the audio loop indefinitely.
 */
void* run_audio_loop(void* ignored) {
    // Create an audio format description.
    AudioStreamBasicDescription fmt = { 0 };
    fmt.mSampleRate = 44100;
    fmt.mFormatID = kAudioFormatLinearPCM;
    fmt.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    fmt.mFramesPerPacket = 1;
    fmt.mChannelsPerFrame = 1; // 2 for stereo
    fmt.mBytesPerPacket = fmt.mBytesPerFrame = 2; // x2 for stereo
    fmt.mBitsPerChannel = 16;

    AudioQueueRef queue;

    // Double buffering. 
    AudioQueueBufferRef buf_ref1, buf_ref2;
    
    int buffer_size_bytes = AUDIO_BUFFER_SIZE * 2;
    
    // Create an audio queue output with the specified format.
    // Third argument is an optional pointer to pass to the callback function.
    if (AudioQueueNewOutput(&fmt, callback, NULL, CFRunLoopGetCurrent(), kCFRunLoopCommonModes, 0, &queue) != 0
        || AudioQueueAllocateBuffer (queue, buffer_size_bytes, &buf_ref1) != 0
        || AudioQueueAllocateBuffer (queue, buffer_size_bytes, &buf_ref2) != 0
    ) {
        fprintf(stderr, "WARNING: Failed to create audio output. No audio will be produced");
        return NULL;
    }
    // Convert reference to a C pointer.
    AudioQueueBuffer* buf1 = buf_ref1;
    AudioQueueBuffer* buf2 = buf_ref2;
    
    // Set buffer size
    buf1->mAudioDataByteSize = buffer_size_bytes;
    buf2->mAudioDataByteSize = buffer_size_bytes;
    
    // Put both buffers in the queue.
    callback (NULL, queue, buf_ref1);
    callback (NULL, queue, buf_ref2);
    // At this point, next_buffer_start_time == start time of the model.
    
    // Set the second buffer to be the one being currently written into.
    next_buffer = buf2->mAudioData;
    
    // Set the volume. (Ignoring errors)
    AudioQueueSetParameter (queue, kAudioQueueParam_Volume, 1.0);
    
    // Start audio at start time plus one buffer duration.
    struct AudioTimeStamp time_stamp = { 0 };
    time_stamp.mHostTime = next_buffer_start_time + BUFFER_DURATION_NS;
    
    // Start as soon as possible.
    if (AudioQueueStart (queue, &time_stamp) != 0) {
        fprintf(stderr, "WARNING: Failed to start audio output. No audio will be produced");
    }
    CFRunLoopRun();
    return NULL;
}

pthread_t loop_thread_id;
bool loop_thread_started = false;

/**
 * Start an audio loop thread that becomes ready to receive
 * audio amplitude samples via add_to_sound(). If there is
 * already an audio loop running, then do nothing.
 * @param start_time The logical time that aligns with the
 *  first audio buffer.
 */
void lf_start_audio_loop(instant_t start_time) {
    
    if (loop_thread_started) return;
    loop_thread_started = true;
    
    // Set the start time of the current buffer to the current time
    // minus twice the buffer duration. The two calls to callback()
    // during setup will increment this to equal to the start time.
    // Then create a thread to
    // start the audio loop. That thread will place
    // two empty audio buffers in the queue and will schedule the
    // audio to start at the current logical time plus the buffer
    // duration. The current buffer being filled (the second buffer)
    // will have logical start time 0, but will play later by less
    // than the buffer duration.
    next_buffer_start_time = start_time - 2 * BUFFER_DURATION_NS;
    
    // Start the audio loop thread.
    pthread_create(&loop_thread_id, NULL, &run_audio_loop, NULL);
}

/**
 * Stop the audio loop thread.
 */
void lf_stop_audio_loop() {
    CFRunLoopStop(CFRunLoopGetCurrent());
}

/**
 * Play the specified waveform with the specified emphasis at
 * the specified time. If the waveform is null, play a simple tick
 * (an impulse). If the waveform has length zero or volume 0,
 * play nothing.
 * 
 * If the time is too far in the future
 * (beyond the window of the current audio write buffer), then
 * block until the audio output catches up. If the audio playback
 * has already passed the specified point, then play the waveform
 * as soon as possible and return 1.
 * Otherwise, return 0.
 * 
 * @param waveform The waveform to play or NULL to just play a tick.
 * @param emphasis The emphasis (0.0 for silence, 1.0 for waveform volume).
 * @param start_time The time to start playing the waveform.
 */
int lf_play_audio_waveform(lf_waveform_t* waveform, float emphasis, instant_t start_time) {
    int result = 0;
    pthread_mutex_lock(&lf_audio_mutex);
    
    // If the buffer into which to write has not yet been set up, wait.
    while (next_buffer == NULL) {
        pthread_cond_wait(&lf_audio_cond, &lf_audio_mutex);
    }
    instant_t time_offset = start_time - next_buffer_start_time;
    
    // If this is late, then tick right away.
    if (time_offset < 0) {
        // printf("WARNING: audio has passed the specified time by %lld.\n", time_offset);
        time_offset = 0;
        result = 1;
    }
    // Calculate the index of the tick.
    size_t index_offset = (time_offset * SAMPLE_RATE) / BILLION;
    
    // If the offset is beyond the end of the audio buffer, then the program
    // has gotten ahead of the audio. Wait for audio to catch up.
    // This happens when a timestamp is at or close to the start time
    // for the buffer because the audio system has not yet invoked the
    // callback to swap buffers.  Here, we wait for the callback to
    // occur.
    while (index_offset >= AUDIO_BUFFER_SIZE) {
        pthread_cond_wait(&lf_audio_cond, &lf_audio_mutex);
        time_offset = get_logical_time() - next_buffer_start_time;
        index_offset = (time_offset * SAMPLE_RATE) / BILLION;
    }
    
    if (waveform == NULL) {
        // Waveform ID is out of range. Just emit a tick.
        add_to_sound(index_offset, MAX_AMPLITUDE * emphasis);
    } else {
        int note_to_use = note_counter++; // Increment so that the next note uses a new slot.
        if (note_counter >= NUM_NOTES) {
            note_counter = 0; // Wrap around.
        }
        // Initialize the note instance to start playing.
        struct note* note_instance = &notes[note_to_use];
        note_instance->waveform = waveform;
        // If the waveform length is 0, do not play anything.
        if (waveform->length > 0) {
            note_instance->volume = emphasis;
            note_instance->position = 0;
                        
            // Add as much of the note instance into the buffer as will fit.
            for (int i = index_offset; i < AUDIO_BUFFER_SIZE; i++) {
                // Calculate the value to add to the sound by averaging all the channels.
                int value = 0;
                for (int channel = 0; channel < waveform->num_channels; channel++) {
                    value += waveform->waveform[note_instance->position + channel];
                }
                value = value / waveform->num_channels;
                add_to_sound(i, value * emphasis);
            
                note_instance->position += note_instance->waveform->num_channels;
                if (note_instance->position >= note_instance->waveform->length - note_instance->waveform->num_channels) {
                    // Reached the end of the note. Reset the note.
                    note_instance->volume = 0.0;
                    note_instance->position = 0;
                    break;
                }
            }
        }
    }
    pthread_mutex_unlock(&lf_audio_mutex);
    return result;
}
