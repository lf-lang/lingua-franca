
#include <time.h>

/**
 * Calculate the necessary offset to bring _LF_CLOCK in parity with the epoch
 * time.
 */
long long calculate_epoch_offset(int clock_id) {
    long long epoch_offset;
    if (clock_id == CLOCK_REALTIME) {
        // Set the epoch offset to zero (see tag.h)
        epoch_offset = 0LL;
    } else {
        // Initialize _lf_epoch_offset to the difference between what is
        // reported by whatever clock LF is using (e.g. CLOCK_MONOTONIC) and
        // what is reported by CLOCK_REALTIME.
        struct timespec physical_clock_snapshot, real_time_start;

        clock_gettime(clock_id, &physical_clock_snapshot);
        long long physical_clock_snapshot_ns = physical_clock_snapshot.tv_sec * 1000000000 + physical_clock_snapshot.tv_nsec;

        clock_gettime(CLOCK_REALTIME, &real_time_start);
        long long real_time_start_ns = real_time_start.tv_sec * 1000000000 + real_time_start.tv_nsec;

        epoch_offset = real_time_start_ns - physical_clock_snapshot_ns;
    }
    return epoch_offset;
}