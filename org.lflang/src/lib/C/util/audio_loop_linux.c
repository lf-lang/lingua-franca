/**
 * @file
 * @author Edward A. Lee
 * @author Soroush Bateni
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
 * Audio functions for Linux.
 * 
 * See audio_loop.h for instructions.
 * 
 * Help from http://equalarea.com/paul/alsa-audio.html
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include "audio_loop.h"
#include <unistd.h>
#include <poll.h>
#include <alsa/asoundlib.h>
#include <stdbool.h>

// Audio device to use for playback
#define AUDIO_DEVICE "default"

pthread_mutex_t lf_audio_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t lf_audio_cond = PTHREAD_COND_INITIALIZER;

// Pointer to the buffer into which to currently write.
// This is null before the buffer is ready.
int16_t* next_buffer = NULL;
instant_t next_buffer_start_time = NEVER;

snd_pcm_t *playback_handle;
snd_async_handler_t *pcm_callback;

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
 * @param playback_handle Handle for the audio interface
 * @param buffer_ref Reference to the buffer of size AUDIO_BUFFER_SIZE to be copied to the hardware
 */
int callback (snd_pcm_t *playback_handle,  int16_t buf_ref[]) {
    int error_number;
    pthread_mutex_lock(&lf_audio_mutex);

    // next_buffer = buf_ref;
    next_buffer = buf_ref;
    // memset(next_buffer, 0, AUDIO_BUFFER_SIZE * sizeof(int16_t));

    // Clear out the next buffer.
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
    if ((error_number = snd_pcm_writei(playback_handle, buf_ref, AUDIO_BUFFER_SIZE)) < 0) {
        error_print("Writing to sound buffer failed: %s", snd_strerror(error_number));
    }

    // In case the other thread is waiting for this event, notify
    // (the other thread should not be waiting).
    pthread_cond_signal(&lf_audio_cond);
    pthread_mutex_unlock(&lf_audio_mutex);
    return error_number;
}

bool stop_audio = false;

/**
 * Run the audio loop indefinitely.
 */
void* run_audio_loop(void* ignored) {
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_sframes_t frames_to_deliver;
    int error_number;
    unsigned int sample_rate = SAMPLE_RATE;
    const char* device_name = AUDIO_DEVICE;
    int buffer_size_bytes = AUDIO_BUFFER_SIZE * 4 * NUM_CHANNELS;

    if ((error_number = snd_pcm_open(&playback_handle, device_name, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        error_print_and_exit("Cannot open audio device %s (%s)\n",
                AUDIO_DEVICE,
             snd_strerror(error_number));
    }

    if ((error_number = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
        error_print_and_exit("Cannot allocate hardware parameter structure (%s)\n",
             snd_strerror(error_number));
    }

    if ((error_number = snd_pcm_hw_params_any(playback_handle, hw_params)) < 0) {
        error_print_and_exit("Cannot initialize hardware parameter structure (%s)\n",
             snd_strerror(error_number));
    }

    if ((error_number = snd_pcm_hw_params_set_access(playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        error_print_and_exit("Cannot set access type (%s)\n",
             snd_strerror(error_number));
    }

    if ((error_number = snd_pcm_hw_params_set_format(playback_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
        error_print_and_exit("Cannot set sample format (%s)\n",
             snd_strerror(error_number));
    }

    if ((error_number = snd_pcm_hw_params_set_rate_near(playback_handle, hw_params, &sample_rate, 0)) < 0) {
        error_print_and_exit("Cannot set sample rate (%s)\n",
             snd_strerror(error_number));
    }
    // FIXME: check sample rate

    if ((error_number = snd_pcm_hw_params_set_channels(playback_handle, hw_params, NUM_CHANNELS)) < 0) {
        error_print_and_exit("Cannot set channel count (%s)\n",
             snd_strerror(error_number));
    }
    snd_pcm_uframes_t periods = buffer_size_bytes / AUDIO_BUFFER_SIZE;
    if ((error_number = snd_pcm_hw_params_set_periods(playback_handle, hw_params, periods, 0)) < 0) {
        error_print_and_exit("Cannot set channel count (%s)\n",
             snd_strerror(error_number));
    }
    snd_pcm_uframes_t size = buffer_size_bytes;
    if ((error_number = snd_pcm_hw_params_set_buffer_size_near(playback_handle, hw_params, &size)) < 0) {
        error_print_and_exit("Cannot set channel count (%s)\n",
             snd_strerror(error_number));
    }
    if ((error_number = snd_pcm_hw_params(playback_handle, hw_params)) < 0) {
        error_print_and_exit("Cannot set parameters (%s)\n",
             snd_strerror(error_number));
    }

    snd_pcm_hw_params_free(hw_params);

    /* tell ALSA to wake us up whenever 4096 or more frames
       of playback data can be delivered. Also, tell
       ALSA that we'll start the device ourselves.
    */

    if ((error_number = snd_pcm_sw_params_malloc(&sw_params)) < 0) {
        error_print_and_exit("Cannot allocate software parameters structure (%s)\n",
             snd_strerror (error_number));
    }
    if ((error_number = snd_pcm_sw_params_current(playback_handle, sw_params)) < 0) {
        error_print_and_exit("Cannot initialize software parameters structure (%s)\n",
             snd_strerror (error_number));
    }
    if ((error_number = snd_pcm_sw_params_set_avail_min(playback_handle, sw_params, buffer_size_bytes)) < 0) {
        error_print_and_exit("Cannot set minimum available count (%s)\n",
             snd_strerror (error_number));
    }
    if ((error_number = snd_pcm_sw_params_set_start_threshold(playback_handle, sw_params, AUDIO_BUFFER_SIZE)) < 0) {
        error_print_and_exit("Cannot set start mode (%s)\n",
             snd_strerror (error_number));
    }
    if ((error_number = snd_pcm_sw_params(playback_handle, sw_params)) < 0) {
        error_print_and_exit("Cannot set software parameters (%s)\n",
             snd_strerror (error_number));
    }

    snd_pcm_sw_params_free(sw_params);

    /*
     * The interface will interrupt the kernel every AUDIO_BUFFER_SIZE frames, and ALSA
     * will wake up this program very soon after that.
    */

    if ((error_number = snd_pcm_prepare(playback_handle)) < 0) {
        error_print_and_exit("Cannot prepare audio interface for use (%s)\n",
             snd_strerror (error_number));
    }


    int16_t buffer[buffer_size_bytes];
    memset(buffer, 0, buffer_size_bytes * sizeof(int16_t));
    int head = 0;
    while (!stop_audio) {
        /*
         * Wait until the interface is ready for data, or BUFFER_DURATION_NS
         * has elapsed.
        */

        if ((error_number = snd_pcm_wait(playback_handle, BUFFER_DURATION_NS/1000)) < 0) {
            error_print("Poll failed (%s)\n", strerror(errno));
            break;
        }

        /* Find out how much space is available for playback data */

        if ((frames_to_deliver = snd_pcm_avail_update(playback_handle)) < 0) {
            if (frames_to_deliver == -EPIPE) {
                error_print("An xrun occured\n");
                continue;
            } else {
                error_print("Unknown ALSA avail update return value (%d)\n",
                     frames_to_deliver);
                break;
            }
        }

        if (frames_to_deliver < AUDIO_BUFFER_SIZE) {
            continue;
        }

        /* deliver the data */
        callback(playback_handle, &(buffer[head]));


        if (head <= (buffer_size_bytes - (2 * AUDIO_BUFFER_SIZE))) {
            head += AUDIO_BUFFER_SIZE;
        } else {
            head = 0;
        }
        // Clear out the next buffer.
        memset(&(buffer[head]), 0, AUDIO_BUFFER_SIZE * sizeof(int16_t));
        next_buffer = &(buffer[head]);
    }

    snd_pcm_close(playback_handle);


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
    stop_audio = true;
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
