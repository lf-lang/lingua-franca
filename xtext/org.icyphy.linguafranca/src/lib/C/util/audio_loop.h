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
 * Audio functions for Linux. To start an audio loop, call
 * `lf_start_audio_loop`, passing it the logical time at which
 * you would like the loop to start.  To play a waveform,
 * call `lf_play_audio_waveform()`.  A waveform may be
 * synthesized or read from a .wav file using 
 * `read_wave_file()` (see wave_file_reader.h).
 * 
 * To use this, include the following flags in your target properties:
 * <pre>
 * target C {
 *     flags: "-lasound -lm",
 *     files: ["/lib/C/util/audio_loop_linux.c", "/lib/C/util/audio_loop.h"]
 * };
 * </pre>
 * 
 * In addition, you need this in your Lingua Franca file:
 * <pre>
 * preamble {=
 *     #include "audio_loop_linux.c"
 * =}
 * </pre>
 */

#ifndef AUDIO_LOOP_H
#define AUDIO_LOOP_H

#include "wave_file_reader.h" // Defines lf_waveform_t.
#include "core/reactor.h"     // Defines instant_t.

// Constants for playback. These are all coupled.
#define SAMPLE_RATE 44100
#define AUDIO_BUFFER_SIZE  4410  // 1/10 second, 100 msec
#define START_THRESHOLD AUDIO_BUFFER_SIZE
#define BUFFER_DURATION_NS 100000000LL
#define NUM_CHANNELS 1 // 2 for stereo

#define MAX_AMPLITUDE 32765

#define NUM_NOTES 8  // Maximum number of notes that can play simultaneously.

/**
 * Start an audio loop thread that becomes ready to receive
 * audio amplitude samples via add_to_sound(). If there is
 * already an audio loop running, then do nothing.
 * @param start_time The logical time that aligns with the
 *  first audio buffer.
 */
void lf_start_audio_loop(instant_t start_time);

/**
 * Stop the audio loop thread.
 */
void lf_stop_audio_loop();

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
int lf_play_audio_waveform(lf_waveform_t* waveform, float emphasis, instant_t start_time);

#endif // AUDIO_LOOP_H
