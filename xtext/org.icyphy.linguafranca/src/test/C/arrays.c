// Library type supporting array(T) declarations.
typedef struct token_t token_t;
struct token_t {
    void* value;
    int element_size;
    int length;
    int initial_ref_count;
    int ref_count;
};

// Library function for allocating memory for an array output.
// This turns over "ownership" of the allocated memory to the output.
void* __new_array_impl(token_t* token, int length) {
    // FIXME: Error checking needed.
    token->value = malloc(token->element_size * length);
    token->ref_count = token->initial_ref_count;
    printf("****** Allocated array with starting ref_count = %d.\n", token->ref_count);
    token->length = length;
    return token->value;
}

// Library function to decrement the reference count and free
// the memory, if appropriate.
void __done_using(token_t* token) {
    token->ref_count--;
    printf("****** After reacting, ref_count = %d.\n", token->ref_count);
    if (token->ref_count == 0) {
        printf("****** Freeing allocated memory.\n");
        free(token->value);
    }
}

// Library function for returning a writable copy of a token.
// If the reference count is 1, it returns the original rather than a copy.
void* __writable_copy_impl(token_t* token) {
    printf("****** Requesting writable copy with reference count %d.\n", token->ref_count);
    if (token->ref_count == 1) {
        printf("****** Avoided copy because reference count is exactly one.\n");
        // Decrement the reference count to avoid the automatic free().
        token->ref_count--;
        return token->value;
    } else {
        printf("****** Copying array because reference count is not one.\n");
        int size = token->element_size * token->length;
        void* copy = malloc(size);
        memcpy(copy, token->value, size);
        return copy;
    }
}

/** set_array() is a macro that hands to an output a dynamically allocated array. */
#define set_array(out, val, len) out->value = val; out->length = len; out->ref_count = out->initial_ref_count; (*out ## _is_present) = true;

/** new_array() is a macro that calls new_array_impl and sets the _is_present variable. */
#define new_array(out, length) __new_array_impl(out, length); (*out ## _is_present) = true;

/** A macro that converts an input name into a reference in the self struct. */
#define writable_copy(input) __writable_copy_impl(self->__ ## input)
