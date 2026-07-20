// ir_rtt_UART_b.c
// B side: wait for 256 bytes of UART data on RX_GPIO,
// immediately reply (echo) with the same data on TX_GPIO (modulated at 38kHz).
// Compile: gcc -O2 -Wall -pthread -o ir_rtt_UART_b ir_rtt_UART_b.c -lpigpio -lrt -lm
// Run:     sudo ./ir_rtt_UART_b

#include <pigpio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#define TX_GPIO 17
#define RX_GPIO 10
#define BAUD_RATE 2400
#define DATA_SIZE 32

static volatile int running = 1;

static void on_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Converts a buffer of bytes into a 38kHz modulated UART wave
static int build_modulated_uart_wave(int gpio, int baud, const uint8_t *buf, int len)
{
    int max_pulses = len * 320;
    gpioPulse_t *pulses = calloc((size_t)max_pulses, sizeof(gpioPulse_t));
    if (!pulses) {
        return -1;
    }

    int pulse_idx = 0;
    const int half_period_us = 13; // approx 38.46 kHz
    const int cycles_per_bit = 16; // 16 * 26 = 416 us (~2400 baud)

    for (int i = 0; i < len; i++) {
        uint8_t byte = buf[i];
        // 10 bits: Start bit (0), 8 data bits (LSB first), Stop bit (1)
        for (int bit = 0; bit < 10; bit++) {
            int bit_val;
            if (bit == 0) {
                bit_val = 0; // Start bit
            } else if (bit == 9) {
                bit_val = 1; // Stop bit
            } else {
                bit_val = (byte >> (bit - 1)) & 1; // Data bits LSB first
            }

            if (bit_val == 0) {
                // Modulated 38kHz carrier
                for (int c = 0; c < cycles_per_bit; c++) {
                    pulses[pulse_idx].gpioOn = 1u << gpio;
                    pulses[pulse_idx].gpioOff = 0;
                    pulses[pulse_idx].usDelay = half_period_us;
                    pulse_idx++;

                    pulses[pulse_idx].gpioOn = 0;
                    pulses[pulse_idx].gpioOff = 1u << gpio;
                    pulses[pulse_idx].usDelay = half_period_us;
                    pulse_idx++;
                }
            } else {
                // Idle / Low (No carrier)
                pulses[pulse_idx].gpioOn = 0;
                pulses[pulse_idx].gpioOff = 1u << gpio;
                pulses[pulse_idx].usDelay = cycles_per_bit * 2 * half_period_us; // 416 us
                pulse_idx++;
            }
        }
    }

    gpioWaveClear();
    gpioWaveAddGeneric(pulse_idx, pulses);
    free(pulses);

    int wave_id = gpioWaveCreate();
    return wave_id;
}

// Sends a buffer in chunks of 16 bytes to fit within pigpio wave limits
static int send_modulated_uart(int gpio, int baud, const uint8_t *buf, int len)
{
    const int chunk_size = 32;
    for (int i = 0; i < len; i += chunk_size) {
        int current_chunk = len - i;
        if (current_chunk > chunk_size) {
            current_chunk = chunk_size;
        }

        int wave_id = build_modulated_uart_wave(gpio, baud, buf + i, current_chunk);
        if (wave_id < 0) {
            fprintf(stderr, "Failed to build wave at index %d\n", i);
            return -1;
        }

        int rc = gpioWaveTxSend(wave_id, PI_WAVE_MODE_ONE_SHOT);
        if (rc < 0) {
            fprintf(stderr, "gpioWaveTxSend failed: %d\n", rc);
            gpioWaveDelete(wave_id);
            return -1;
        }

        while (gpioWaveTxBusy()) {
            gpioDelay(500); // check every 0.5ms
        }

        gpioWaveDelete(wave_id);

        // Add a 20ms gap between chunks to allow the receiver's AGC to reset
        if (i + chunk_size < len) {
            gpioDelay(20000); // 20ms
        }
    }
    return 0;
}

int main(void)
{
    signal(SIGINT, on_sigint);

    if (gpioInitialise() < 0) {
        fprintf(stderr, "gpioInitialise failed. Try running with sudo.\n");
        return 1;
    }

    gpioSetMode(TX_GPIO, PI_OUTPUT);
    gpioWrite(TX_GPIO, 0);

    gpioSetMode(RX_GPIO, PI_INPUT);
    gpioSetPullUpDown(RX_GPIO, PI_PUD_UP);

    // Open software serial on RX_GPIO
    int rc = gpioSerialReadOpen(RX_GPIO, BAUD_RATE, 8);
    if (rc < 0) {
        fprintf(stderr, "gpioSerialReadOpen failed: %d\n", rc);
        gpioTerminate();
        return 1;
    }

    printf("B responder started. TX GPIO=%d, RX GPIO=%d, baud=%d\n", TX_GPIO, RX_GPIO, BAUD_RATE);
    printf("Waiting for IR data...\n");

    uint8_t rx_buf[1024];
    int rx_len = 0;
    uint32_t last_byte_tick = 0;

    while (running) {
        char chunk[256];
        int read_rc = gpioSerialRead(RX_GPIO, chunk, sizeof(chunk));
        if (read_rc > 0) {
            if (rx_len + read_rc <= (int)sizeof(rx_buf)) {
                memcpy(rx_buf + rx_len, chunk, (size_t)read_rc);
                rx_len += read_rc;
            }
            last_byte_tick = gpioTick();
        } else if (read_rc < 0) {
            fprintf(stderr, "gpioSerialRead error: %d\n", read_rc);
            break;
        }

        if (rx_len > 0) {
            uint32_t now = gpioTick();
            uint32_t elapsed_us = now - last_byte_tick;

            // Echo back if we received the expected block size (256 bytes)
            // OR if we received some bytes and experienced an idle timeout (100ms)
            if (rx_len >= DATA_SIZE || elapsed_us > 100000) {
                printf("Received %d bytes. Echoing back...", rx_len);
                fflush(stdout);

                // Small delay to let A's side settle into read mode
                gpioDelay(1000); // 1ms

                if (send_modulated_uart(TX_GPIO, BAUD_RATE, rx_buf, rx_len) < 0) {
                    printf(" Echo failed.\n");
                } else {
                    printf(" Echoed successfully.\n");
                }
                fflush(stdout);

                // Reset buffer
                rx_len = 0;
                last_byte_tick = 0;
            }
        }

        gpioDelay(1000); // 1ms sleep to save CPU
    }

    gpioSerialReadClose(RX_GPIO);
    gpioWrite(TX_GPIO, 0);
    gpioTerminate();

    return 0;
}