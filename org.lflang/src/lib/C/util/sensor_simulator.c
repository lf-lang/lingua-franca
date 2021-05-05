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
#include "core/util.h"
#include "core/platform.h"

_lf_thread_t _lf_sensor_thread_id;
int _lf_sensor_thread_created = 0;

// Maximum number of milliseconds that wgetchr will block for.
#define WGETCHR_TIMEOUT 1000

// Support ASCII characters SPACE (32) through DEL (127).
#define LF_SENSOR_TRIGGER_TABLE_SIZE 96

trigger_t* trigger_table[LF_SENSOR_TRIGGER_TABLE_SIZE];

// The newline character '\n', which is platform dependent, is
// handled specially.
trigger_t* _lf_sensor_newline_trigger = NULL;

// Trigger for any key.
trigger_t* _lf_sensor_any_key_trigger = NULL;

SCREEN* screen;

lf_mutex_t _lf_sensor_mutex;

lf_cond_t _lf_sensor_simulator_started;

/**
 * Default window from which to get input characters.
 * If show_welcome_message() is called, this will be the welcome
 * message window. Otherwise, it will be stdscr, the default
 * curses window.
 */
WINDOW* _lf_sensor_default_window;

/** Tick window. */
WINDOW* _lf_sensor_tick_window;

/**
 * Keep track of the tick cursor position directly so it
 * doesn't get as messed up by printf() calls.
 */
int _lf_sensor_tick_cursor_x, _lf_sensor_tick_cursor_y;

/** Print window. */
WINDOW* _lf_sensor_print_window;

/** The print cursor y position to wrap it around to the top. */
int _lf_sensor_print_cursor_y;

/** The print window height. */
int _lf_sensor_print_window_height;

/**
 * Thread to read input characters until an EOF is received.
 * For each character received, if there is a registered trigger
 * for that character, schedule that trigger with a payload
 * equal to the character that was typed.
 * Otherwise, the character is ignored.
 */
void* read_input(void* ignored) {
    while(_lf_sensor_thread_created != 0) {
        int c = wgetch(_lf_sensor_default_window);
        if (c == EOF) {
            // End of file received. Exit thread.
            _lf_sensor_thread_created = 0;
        } else if (c != ERR) {
            // wgetch returns ERR if it times out, in which case, we continue
            // and check whether _lf_sensor_thread_created has been set to 0.
            // So here, ERR was not returned.
            // It is imperative that we not hold the _lf_sensor_mutex when
            // calling schedule(), because schedule() acquires another mutex.
            // We would create a deadlock risk.  The following code is correct
            // because a trigger_table entry, once assigned a value, becomes
            // immutable.
            if (c == '\n' && _lf_sensor_newline_trigger != NULL) {
                schedule_copy(_lf_sensor_newline_trigger, 0, &c, 1);
            } else if (c - 32 >= 0 && c - 32 < LF_SENSOR_TRIGGER_TABLE_SIZE && trigger_table[c-32] != NULL) {
                schedule_copy(trigger_table[c-32], 0, &c, 1);
            }
            // Any key trigger triggers after specific keys.
            if (_lf_sensor_any_key_trigger != NULL) {
                schedule_copy(_lf_sensor_any_key_trigger, 0, &c, 1);
            }
        }
    }
    _lf_sensor_thread_created = 0;
    return NULL;
}

/**
 * End ncurses control of the terminal.
 */
void end_ncurses() {
    lf_mutex_lock(&_lf_sensor_mutex);
    register_print_function(NULL);
    endwin();
    _lf_sensor_thread_created = 0;
    lf_mutex_unlock(&_lf_sensor_mutex);
}

/**
 * Put a persistent message in the upper left of the terminal window.
 * The message will be left justified, with each string
 * in the specified array on a new line.
 * This should not be called directly by the user.
 * It assumes the mutex lock is held.
 * @param message_lines The message lines.
 * @param number_of_lines The number of lines.
 */
void _lf_show_message(char* message_lines[], int number_of_lines) {
    int term_height, term_width;
    int message_width = 0;
    // Find the widest message in the list.
    for (int i = 0; i < number_of_lines; i++) {
        size_t width = strlen(message_lines[i]);
        if (width > message_width) {
            message_width = width;
        }
    }
    getmaxyx(stdscr, term_height, term_width);   // Get the size of the terminal window.
    WINDOW* center_win = newwin(number_of_lines + 2, message_width + 2, 0, 0);
    box(center_win, 0, 0);
    wrefresh(center_win);

    // wattron(center_win, COLOR_PAIR(2));

    for (int i = 0; i < number_of_lines; i++) {
        mvwprintw(center_win, i + 1, 1, "%s", message_lines[i]);
        // According to curses docs, the following should not be necessary
        // after each print. But if I wait and do it later, the output
        // gets garbled.
        wrefresh(center_win);
    }
    _lf_sensor_default_window = center_win;
}

/**
 * Start a tick window on the right of the terminal window.
 * This should not be called directly by the user.
 * It assumes the mutex lock is held.
 * @param width The width of the window.
 */
void _lf_start_tick_window(int width) {
    int term_height, term_width;
    getmaxyx(stdscr, term_height, term_width);   // Get the size of the terminal window.
    _lf_sensor_tick_window = newwin(term_height, width + 2, 0, term_width - width - 2);
    box(_lf_sensor_tick_window, 0, 0);
    wrefresh(_lf_sensor_tick_window);
    wmove(_lf_sensor_tick_window, 1, 1);  // Ensure to not overwrite the box.
    _lf_sensor_tick_cursor_x = _lf_sensor_tick_cursor_y = 1;
    move(0, 0);
}

/**
 * Start a window on the bottom left of the terminal window
 * for printed messages from the application.
 * This should not be called directly by the user.
 * It assumes the mutex lock is held.
 * @param above Space to leave above the window.
 * @param right Space to leave to the right of the window.
 */
void _lf_start_print_window(int above, int right) {
    int term_height, term_width;
    getmaxyx(stdscr, term_height, term_width);   // Get the size of the terminal window.
    _lf_sensor_print_window_height = term_height - above;
    _lf_sensor_print_window = newwin(_lf_sensor_print_window_height, term_width - right, above, 0);
    wrefresh(_lf_sensor_print_window);
    wmove(_lf_sensor_print_window, 0, 0);
    _lf_sensor_print_cursor_y = 0;
}

/**
 * Function to register to handle printing of messages in util.h/c.
 * This acquires the mutex lock.
 */
void _lf_print_message_function(char* format, va_list args) {
    // Replace the last \n with a \0 in the format and handle trailing returns manually.
    char* newline = strrchr(format, '\n');
    if (newline != NULL) {
        (*newline) = '\0';
    }
    lf_mutex_lock(&_lf_sensor_mutex);

    // If the sensor simulator has not started yet, wait for it to start.
    while (!_lf_sensor_thread_created) {
        lf_cond_wait(&_lf_sensor_simulator_started, &_lf_sensor_mutex);
    }

    vwprintw(_lf_sensor_print_window, format, args);
    int y, x;
    getyx(_lf_sensor_print_window, y, x);
    if (newline != NULL) {
        x = 0;
        y++;
    }
    if (y >= _lf_sensor_print_window_height - 1) {
        y = 0;
    }
    wmove(_lf_sensor_print_window, y, x);
    if (newline != NULL) {
        wclrtoeol(_lf_sensor_print_window);
    }

    wrefresh(_lf_sensor_print_window);
    lf_mutex_unlock(&_lf_sensor_mutex);
}

/**
 * Start the sensor simulator if it has not been already
 * started. This must be called at least once before any
 * call to register_sensor_key.
 * @return 0 for success, error code for failure.
 * @param message_lines The message lines.
 * @param number_of_lines The number of lines.
 * @param tick_window_width The width of the tick window or 0 for none.
 */
int start_sensor_simulator(char* message_lines[], int number_of_lines, int tick_window_width) {
    lf_mutex_init(&_lf_sensor_mutex);
    lf_mutex_lock(&_lf_sensor_mutex);
    int result = 0;
    if (_lf_sensor_thread_created == 0) {
        // Thread has not been created.
        // Zero out the trigger table.
        for (int i = 0; i < LF_SENSOR_TRIGGER_TABLE_SIZE; i++) {
            trigger_table[i] = NULL;
        }
        if (atexit(end_ncurses) != 0) {
            warning_print("sensor_simulator: Failed to register end_ncurses function!");
        }
        // Clean up any previous curses state.
        if (!isendwin()) {
            endwin();
        }

        // Initialize ncurses.
        DEBUG_PRINT("Initializing ncurses.");
        initscr();
        start_color();     // Allow colors.
        noecho();          // Don't echo input
        cbreak();          // Don't wait for Return or Enter
        wtimeout(stdscr, WGETCHR_TIMEOUT); // Don't wait longer than this
        refresh();         // Not documented, but needed?

        _lf_sensor_default_window = stdscr;
        if (message_lines != NULL && number_of_lines > 0) {
            _lf_show_message(message_lines, number_of_lines);
        }
        _lf_sensor_tick_window = stdscr;
        if (tick_window_width > 0) {
            _lf_start_tick_window(tick_window_width);
        }
        _lf_start_print_window(number_of_lines + 2, tick_window_width + 2);

        // Create the thread that listens for input.
        int result = lf_thread_create(&_lf_sensor_thread_id, &read_input, NULL);
        if (result == 0) {
            register_print_function(&_lf_print_message_function);
            _lf_sensor_thread_created = 1;
        } else {
            error_print("Failed to start sensor simulator!");
        }
        lf_cond_broadcast(&_lf_sensor_simulator_started);
    }
    lf_mutex_unlock(&_lf_sensor_mutex);
    DEBUG_PRINT("Finished starting sensor simulator.");
    return result;
}

/**
 * Place a tick (usually a single character) in the tick window.
 * @param character The tick character.
 */
void show_tick(char* character) {
    lf_mutex_lock(&_lf_sensor_mutex);

    // If necessary, wait for the sensor simulator start.
    while (!_lf_sensor_thread_created) {
        DEBUG_PRINT("Waiting for sensor simulator start.");
        lf_cond_wait(&_lf_sensor_simulator_started, &_lf_sensor_mutex);
    }

    wmove(_lf_sensor_tick_window, _lf_sensor_tick_cursor_y, _lf_sensor_tick_cursor_x);
    wprintw(_lf_sensor_tick_window, character);
    int tick_height, tick_width;
    getmaxyx(_lf_sensor_tick_window, tick_height, tick_width);
    _lf_sensor_tick_cursor_x += strlen(character);
    if (_lf_sensor_tick_cursor_x >= tick_width - 1) {
        _lf_sensor_tick_cursor_x = 1;
        _lf_sensor_tick_cursor_y++;
    }
    if (_lf_sensor_tick_cursor_y >= tick_height - 1) {
        _lf_sensor_tick_cursor_y = 1;
    }
    wmove(_lf_sensor_tick_window, _lf_sensor_tick_cursor_y, _lf_sensor_tick_cursor_x);
    wrefresh(_lf_sensor_tick_window);

    // Move the standard string cursor to 0, 0, so printf()
    // calls don't mess up the screen as much.
    wmove(stdscr, 0, 0);
    refresh();
    lf_mutex_unlock(&_lf_sensor_mutex);
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
    if (key != '\n' && key != '\0' && (index < 0 || index >= LF_SENSOR_TRIGGER_TABLE_SIZE)) {
        return 2;
    }
    int result = 0;
    lf_mutex_lock(&_lf_sensor_mutex);
    if (key == '\n') {
        if (_lf_sensor_newline_trigger != NULL) {
            result = 1;
        } else {
            _lf_sensor_newline_trigger = action;
        }
    } else if (key == '\0') {
        // Any key trigger.
        if (_lf_sensor_any_key_trigger != NULL) {
            result = 1;
        } else {
            _lf_sensor_any_key_trigger = action;
        }
    } else if (trigger_table[index] != NULL) {
        result = 1;
    } else {
        trigger_table[index] = action;
    }
    lf_mutex_unlock(&_lf_sensor_mutex);
    return result;
}
