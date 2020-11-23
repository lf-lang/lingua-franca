/**
@file
@author Edward A. Lee (eal@berkeley.edu)

@section LICENSE
Copyright (c) 2020, The University of California at Berkeley.

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

@section DESCRIPTION

See sensor_simulator.h.
*/

#include <pthread.h>
#include <ncurses.h>
#include "sensor_simulator.h"
#include "ctarget.h"

pthread_t thread_id;
int thread_created = 0;

// Support ASCII characters SPACE (32) through DEL (127).
#define TRIGGER_TABLE_SIZE 96

trigger_t* trigger_table[TRIGGER_TABLE_SIZE];

// The newline character '\n', which is platform dependent, is
// handled specially.
trigger_t* newline_trigger = NULL;

// Trigger for any key.
trigger_t* any_key_trigger = NULL;

pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * Thread to read input characters until an EOF is received.
 * For each character received, if there is a registered trigger
 * for that character, schedule that trigger with a payload
 * equal to the character that was typed.
 * Otherwise, the character is ignored.
 */
void* read_input(void* ignored) {
    int c;
    while((c = getch()) != EOF) {
        // It is imperative that we not hold the sensor_mutex when
        // calling schedule(), because schedule() acquires another mutex.
        // We would create a deadlock risk.  The following code is correct
        // because a trigger_table entry, once assigned a value, becomes
        // immutable.
        if (c == '\n' && newline_trigger != NULL) {
            schedule_copy(newline_trigger, 0, &c, 1);
        } else if (c - 32 >= 0 && c - 32 < TRIGGER_TABLE_SIZE && trigger_table[c-32] != NULL) {
            schedule_copy(trigger_table[c-32], 0, &c, 1);
        }
        // Any key trigger triggers after specific keys.
        if (any_key_trigger != NULL) {
            schedule_copy(any_key_trigger, 0, &c, 1);
        }
    }
    thread_created = 0;
    return NULL;
}

/**
 * End ncurses control of the terminal.
 */
void end_ncurses() {
    endwin();
}

/**
 * Start the sensor simulator if it has not been already
 * started. This must be called at least once before any
 * call to register_sensor_key.
 * @return 0 for success, error code for failure.
 */
int start_sensor_simulator() {
    pthread_mutex_lock(&sensor_mutex);
    int result = 0;
    if (thread_created == 0) {
        // Thread has not been created.
        // Zero out the trigger table.
        for (int i = 0; i < TRIGGER_TABLE_SIZE; i++) {
            trigger_table[i] = NULL;
        }
        // Initialize ncurses.
        initscr();
        noecho();          // Don't echo input
        cbreak();          // Don't wait for Return or Enter

        if (atexit(end_ncurses) != 0) {
            fprintf(stderr, "WARNING: sensor_simulator: Failed to register end_ncurses function!");
        }

        // Create the thread that listens for input.
        int result = pthread_create(&thread_id, NULL, &read_input, NULL);
        if (result == 0) {
            thread_created = 1;
        }
    }
    pthread_mutex_unlock(&sensor_mutex);
    return result;
}

/**
 * Register a keyboard key to trigger the specified action.
 * Printable ASCII characters (codes 32 to 127) are supported
 * plus '\n' and '\0', where the latter registers a trigger
 * to invoked when any key is pressed. If a specific key is
 * registered and any key ('\0') is also registered, the
 * any key trigger will be scheduled after the specific key
 * is scheduled. If these triggers belong to different reactors,
 * they could be invoked in parallel.
 * This will fail if the specified key has already been
 * registered (error code 1), or the key is not a supported key or a
 * newline ‘\n’ or any key '\0' (error code 2) or if the trigger is NULL
 * (error code 3).
 * @param key The key to register.
 * @param action The action to trigger when the key is pressed
 *  (a pointer to a trigger_t struct).
 * @return 0 for success, error code for failure.
 */
int register_sensor_key(char key, void* action) {
    if (action == NULL) {
        return 3;
    }
    int index = key - 32;
    if (key != '\n' && key != '\0' && (index < 0 || index >= TRIGGER_TABLE_SIZE)) {
        return 2;
    }
    int result = 0;
    pthread_mutex_lock(&sensor_mutex);
    if (key == '\n') {
        if (newline_trigger != NULL) {
            result = 1;
        } else {
            newline_trigger = action;
        }
    } else if (key == '\0') {
        // Any key trigger.
        if (any_key_trigger != NULL) {
            result = 1;
        } else {
            any_key_trigger = action;
        }
    } else if (trigger_table[index] != NULL) {
        result = 1;
    } else {
        trigger_table[index] = action;
    }
    pthread_mutex_unlock(&sensor_mutex);
    return result;
}

/**
 * Put a message in the center of the terminal window.
 * The message will be left justified, with each string
 * in the specified array on a new line.
 * The cursor is then moved to the (0,0) position (upper right).
 * @param message_lines The message lines.
 * @param number_of_lines The number of lines.
 */
void show_message(char* message_lines[], int number_of_lines) {
    int term_height, term_width;
    int message_width = 0;
    // Find the widest message in the list.
    for (int i = 0; i < number_of_lines; i++) {
        size_t width = strlen(message_lines[i]);
        if (width > message_width) {
            message_width = width;
        }
    }
    pthread_mutex_lock(&sensor_mutex);
    getmaxyx(stdscr, term_height, term_width);   // Get the size of the terminal window.
    int x = (term_width - message_width)/2;
    int y = (term_height - number_of_lines)/2;
    for (int i = 0; i < number_of_lines; i++) {
        move(y++, x);
        printw("%s", message_lines[i]);
    }
    move(0, 0);
    refresh();
    pthread_mutex_unlock(&sensor_mutex);
}
