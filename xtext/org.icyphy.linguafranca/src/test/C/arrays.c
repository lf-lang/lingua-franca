

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


/** A macro that converts an input name into a reference in the self struct. */
#define writable_copy(input) __writable_copy_impl(self->__ ## input)
