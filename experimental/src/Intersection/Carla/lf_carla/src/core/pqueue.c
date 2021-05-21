/*
 * Copyright (c) 2014, Volkan Yazıcı <volkan.yazici@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Modified by Marten Lohstroh (May, 2019).
 * Changes: 
 * - Require implementation of a pqueue_eq_elem_f function to determine
 *   whether two elements are equal or not; and
 * - The provided pqueue_eq_elem_f implementation is used to test and 
 *   search for equal elements present in the queue; and
 * - Removed capability to reassign priorities.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pqueue.h"
#include "util.h"

#define LF_LEFT(i)   ((i) << 1)
#define LF_RIGHT(i)  (((i) << 1) + 1)
#define LF_PARENT(i) ((i) >> 1)

/**
 * Find an element in the queue that matches the given element up to
 * and including the given maximum priority.
 */ 
void* find_equal(pqueue_t *q, void *e, int pos, pqueue_pri_t max) {
    if (pos < 0) {
        error_print_and_exit("find_equal() called with a negative pos index.");
    }

    // Stop the recursion when we've reached the end of the 
    // queue. This has to be done before accessing the queue
    // to avoid segmentation fault.
    if (!q || (size_t)pos >= q->size) {
        return NULL;
    }

    void* rval;
    void* curr = q->d[pos];

    // Stop the recursion when we've surpassed the maximum priority.
    if (!curr || q->cmppri(q->getpri(curr), max)) {
        return NULL;
    }
    
    if (q->eqelem(curr, e)) {
        return curr;
    } else {
        rval = find_equal(q, e, LF_LEFT(pos), max);
        if (rval) 
            return rval;
        else
            return find_equal(q, e, LF_RIGHT(pos), max);
    }
    return NULL;
}

/**
 * Find an element in the queue that matches the given element up to
 * but not including the given maximum priority. The matching element
 * has to _also_ have the same priority.
 */ 
void* find_equal_same_priority(pqueue_t *q, void *e, int pos) {
    if (pos < 0) {
        error_print_and_exit("find_equal_same_priority() called with a negative pos index.");
    }

    // Stop the recursion when we've reached the end of the 
    // queue. This has to be done before accessing the queue
    // to avoid segmentation fault.
    if (!q || (size_t)pos >= q->size) {
        return NULL;
    }
    
    void* rval;
    void* curr = q->d[pos];

    // Stop the recursion once we've surpassed the priority of the element
    // we're looking for.
    if (!curr || q->cmppri(q->getpri(curr), q->getpri(e))) {
        return NULL;
    }
    
    if (q->getpri(curr) == q->getpri(e) && q->eqelem(curr, e)) {
        return curr;
    } else {
        rval = find_equal_same_priority(q, e, LF_LEFT(pos));
        if (rval) 
            return rval;
        else
            return find_equal_same_priority(q, e, LF_RIGHT(pos));   
    }

    // for (int i=1; i < q->size; i++) {
    //     if (q->d[i] == e) {
    //         return q->d[i];
    //     }
    // }
    return NULL;
}

pqueue_t * pqueue_init(size_t n,
                       pqueue_cmp_pri_f cmppri,
                       pqueue_get_pri_f getpri,
                       pqueue_get_pos_f getpos,
                       pqueue_set_pos_f setpos,
                       pqueue_eq_elem_f eqelem,
                       pqueue_print_entry_f prt) {
    pqueue_t *q;

    if (!(q = (pqueue_t*)malloc(sizeof(pqueue_t))))
        return NULL;

    /* Need to allocate n+1 elements since element 0 isn't used. */
    if (!(q->d = (void**)malloc((n + 1) * sizeof(void *)))) {
        free(q);
        return NULL;
    }

    q->size = 1;
    q->avail = q->step = (n+1);  /* see comment above about n+1 */
    q->cmppri = cmppri;
    q->getpri = getpri;
    q->getpos = getpos;
    q->setpos = setpos;
    q->eqelem = eqelem;
    q->prt = prt;
    return q;
}

void pqueue_free(pqueue_t *q) {
    free(q->d);
    free(q);
}

size_t pqueue_size(pqueue_t *q) {
    // Queue element 0 exists but doesn't count since it isn't used.
    return (q->size - 1);
}

static size_t maxchild(pqueue_t *q, size_t i) {
    size_t child_node = LF_LEFT(i);

    if (child_node >= q->size)
        return 0;

    if ((child_node+1) < q->size &&
        (q->cmppri(q->getpri(q->d[child_node]), q->getpri(q->d[child_node+1])))) 
        child_node++; /* use right child instead of left */

    return child_node;
}

static size_t bubble_up(pqueue_t *q, size_t i) {
    size_t parent_node;
    void *moving_node = q->d[i];
    pqueue_pri_t moving_pri = q->getpri(moving_node);

    for (parent_node = LF_PARENT(i);
         ((i > 1) && q->cmppri(q->getpri(q->d[parent_node]), moving_pri));
         i = parent_node, parent_node = LF_PARENT(i))
    {
        q->d[i] = q->d[parent_node];
        q->setpos(q->d[i], i);
    }

    q->d[i] = moving_node;
    q->setpos(moving_node, i);
    return i;
}

static void percolate_down(pqueue_t *q, size_t i) {
    size_t child_node;
    void *moving_node = q->d[i];
    pqueue_pri_t moving_pri = q->getpri(moving_node);

    while ((child_node = maxchild(q, i)) &&
           q->cmppri(moving_pri, q->getpri(q->d[child_node])))
    {
        q->d[i] = q->d[child_node];
        q->setpos(q->d[i], i);
        i = child_node;
    }

    q->d[i] = moving_node;
    q->setpos(moving_node, i);
}

void* pqueue_find_equal_same_priority(pqueue_t *q, void *e) {
    return find_equal_same_priority(q, e, 1);
}

void* pqueue_find_equal(pqueue_t *q, void *e, pqueue_pri_t max) {
    return find_equal(q, e, 1, max);
}

int pqueue_insert(pqueue_t *q, void *d) {
    void **tmp;
    size_t i;
    size_t newsize;

    if (!q) return 1;

    // printf("==Before insert==\n");
    // pqueue_dump(q, stdout, q->prt);

    // for(int i=1; i < q->size; i++) {
    //     if (d == q->d[i]) {
    //         printf("Duplicate found!\n");
    //         exit(1);
    //     }
    // }

    /* allocate more memory if necessary */
    if (q->size >= q->avail) {
        newsize = q->size + q->step;
        if (!(tmp = (void**)realloc(q->d, sizeof(void *) * newsize)))
            return 1;
        q->d = tmp;
        q->avail = newsize;
    }
    /* insert item and organize the tree */
    i = q->size++;
    q->d[i] = d;
    bubble_up(q, i);

    // printf("==After insert==\n");
    // pqueue_dump(q, stdout, q->prt);


    // NOTE: Only use this for debugging!
    // if (!pqueue_is_valid(q)) {
    //     pqueue_dump(q, stdout, q->prt);
    //     exit(1);
    // }

    return 0;
}

int pqueue_remove(pqueue_t *q, void *d) {
    size_t posn = q->getpos(d);
    q->d[posn] = q->d[--q->size];
    if (q->cmppri(q->getpri(d), q->getpri(q->d[posn])))
        bubble_up(q, posn);
    else
        percolate_down(q, posn);

    return 0;
}

void* pqueue_pop(pqueue_t *q) {
    void* head;
    
    if (!q || q->size == 1)
        return NULL;
        
    head = q->d[1];
    q->d[1] = q->d[--q->size];
    percolate_down(q, 1);
    
    return head;
}

void* pqueue_peek(pqueue_t *q) {
    void *d;
    if (!q || q->size == 1)
        return NULL;
    d = q->d[1];
    return d;
}

void pqueue_dump(pqueue_t *q, pqueue_print_entry_f print) {
    size_t i;

    DEBUG_PRINT("posn\tleft\tright\tparent\tmaxchild\t...");
    for (i = 1; i < q->size ;i++) {
        DEBUG_PRINT("%zu\t%zu\t%zu\t%zu\t%ul\t",
                i,
                LF_LEFT(i), LF_RIGHT(i), LF_PARENT(i),
                (unsigned int)maxchild(q, i));
        print(q->d[i]);
    }
}

void pqueue_print(pqueue_t *q, pqueue_print_entry_f print) {
    pqueue_t *dup;
	void *e;

    dup = pqueue_init(q->size,
                      q->cmppri, q->getpri,
                      q->getpos, q->setpos, q->eqelem, q->prt);
    dup->size = q->size;
    dup->avail = q->avail;
    dup->step = q->step;

    memcpy(dup->d, q->d, (q->size * sizeof(void *)));

    while ((e = pqueue_pop(dup)))
		print(e);

    pqueue_free(dup);
}


static int subtree_is_valid(pqueue_t *q, int pos) {
    if (pos < 0) {
        error_print_and_exit("subtree_is_valid() called with a negative pos index.");
    }

    int left_pos = LF_LEFT(pos);
    if (left_pos < 0) {
        error_print_and_exit("subtree_is_valid(): index overflow detected.");
    }

    if ((size_t)left_pos < q->size) {
        /* has a left child */
        if (q->cmppri(q->getpri(q->d[pos]), q->getpri(q->d[LF_LEFT(pos)])))
            return 0;
        if (!subtree_is_valid(q, LF_LEFT(pos)))
            return 0;
    }

    int right_pos = LF_RIGHT(pos);
    if (right_pos < 0) {
        error_print_and_exit("subtree_is_valid(): index overflow detected.");
    }
    if ((size_t)right_pos < q->size) {
        /* has a right child */
        if (q->cmppri(q->getpri(q->d[pos]), q->getpri(q->d[LF_RIGHT(pos)])))
            return 0;
        if (!subtree_is_valid(q, LF_RIGHT(pos)))
            return 0;
    }
    return 1;
}

int pqueue_is_valid(pqueue_t *q) {
    return subtree_is_valid(q, 1);
}
