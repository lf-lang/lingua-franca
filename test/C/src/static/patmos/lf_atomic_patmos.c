/**
 * @author Soroush Bateni
 * @author Erling Rennemo Jellum
 * @copyright (c) 2023
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 * @brief Implements the atomics API using GCC/Clang APIs.
 */

#include "lf_atomic.h"
#include "platform.h"

// Forward declare the functions for enabling/disabling interrupts. Must be
// implemented in the platform support file of the target.
int lf_disable_interrupts_nested();
int lf_enable_interrupts_nested();

int32_t lf_atomic_fetch_add32(int32_t *ptr, int32_t value) {
    lf_disable_interrupts_nested(); 
    int32_t res = *ptr; 
    *ptr += value; 
    lf_enable_interrupts_nested(); 
    return res;
}

int64_t lf_atomic_fetch_add64(int64_t *ptr, int64_t value) {
    lf_disable_interrupts_nested(); 
    int64_t res = *ptr; 
    *ptr += value; 
    lf_enable_interrupts_nested(); 
    return res;
}

int32_t lf_atomic_add_fetch32(int32_t *ptr, int32_t value) {
    lf_disable_interrupts_nested();
    int res = *ptr + value;
    *ptr = res;
    lf_enable_interrupts_nested();
    return res;
}

int64_t lf_atomic_add_fetch64(int64_t *ptr, int64_t value) {
    lf_disable_interrupts_nested();
    int64_t res = *ptr + value;
    *ptr = res;
    lf_enable_interrupts_nested();
    return res;
}

bool lf_atomic_bool_compare_and_swap32(int32_t *ptr, int32_t oldval, int32_t newval) {
    lf_disable_interrupts_nested();
    bool res = false;
    if ((*ptr) == oldval) {
        *ptr = newval;
        res = true;
    }
    lf_enable_interrupts_nested();
    return res;
}

bool lf_atomic_bool_compare_and_swap64(int64_t *ptr, int64_t oldval, int64_t newval) {
    lf_disable_interrupts_nested();
    bool res = false;
    if ((*ptr) == oldval) {
        *ptr = newval;
        res = true;
    }
    lf_enable_interrupts_nested();
    return res;
}

int32_t  lf_atomic_val_compare_and_swap32(int32_t *ptr, int32_t oldval, int32_t newval) {
    lf_disable_interrupts_nested();
    int res = *ptr;
    if ((*ptr) == oldval) {
        *ptr = newval;
    }
    lf_enable_interrupts_nested();
    return res;
}

int64_t  lf_atomic_val_compare_and_swap64(int64_t *ptr, int64_t oldval, int64_t newval) {
    lf_disable_interrupts_nested();
    int64_t res = *ptr;
    if ((*ptr) == oldval) {
        *ptr = newval;
    }
    lf_enable_interrupts_nested();
    return res;
}