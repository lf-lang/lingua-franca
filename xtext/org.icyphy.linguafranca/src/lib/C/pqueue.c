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
 * - require implementation of a pqueue_eq_elem_f function to determine
 *    whether two elements are equal or not
 * - the pqueue_insert function removes possible duplicate entries after
 *   insertion. Among duplicates, only the most recently added entry is kept.
 * - removed capability to reassign priorities
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pqueue.h"


#define left(i)   ((i) << 1)
#define right(i)  (((i) << 1) + 1)
#define parent(i) ((i) >> 1)


pqueue_t *
pqueue_init(size_t n,
            pqueue_cmp_pri_f cmppri,
            pqueue_get_pri_f getpri,
            pqueue_get_pos_f getpos,
            pqueue_set_pos_f setpos,
            pqueue_eq_elem_f eqelem,
            pqueue_print_entry_f prt)
{
    pqueue_t *q;

    if (!(q = malloc(sizeof(pqueue_t))))
        return NULL;

    /* Need to allocate n+1 elements since element 0 isn't used. */
    if (!(q->d = malloc((n + 1) * sizeof(void *)))) {
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

void
pqueue_free(pqueue_t *q)
{
    free(q->d);
    free(q);
}


size_t
pqueue_size(pqueue_t *q)
{
    /* queue element 0 exists but doesn't count since it isn't used. */
    return (q->size - 1);
}

static size_t
maxchild(pqueue_t *q, size_t i)
{
    size_t child_node = left(i);

    if (child_node >= q->size)
        return 0;

    if ((child_node+1) < q->size &&
        (q->cmppri(q->getpri(q->d[child_node]), q->getpri(q->d[child_node+1])))) 
        child_node++; /* use right child instead of left */

    return child_node;
}

// NOTE: must be run after each insertion (only removes one duplicate).
static void
dedup(pqueue_t *q, size_t i)
{
    size_t parent_node;
    size_t sibling_node;
    void *inserted_node = q->d[i];
    pqueue_pri_t prio = q->getpri(inserted_node);

    // printf("==Before dedup==\n");
    // pqueue_dump(q, stdout, q->prt);
    
    if (i > 1) { // nothing to do if there's only one node
        // look up
        parent_node = parent(i);
        if (q->getpri(q->d[parent_node]) == prio) {
            if (q->eqelem(q->d[parent_node], inserted_node)) {
                pqueue_remove(q, q->d[parent_node]);
            }
        }
    
        // look left
        sibling_node = left(parent_node);
        if (sibling_node != i && sibling_node < q->size 
                && q->getpri(q->d[sibling_node]) == prio) {
            if (q->eqelem(q->d[sibling_node], inserted_node)) {
                pqueue_remove(q, q->d[sibling_node]);
            }
        }

        // look right
        sibling_node = right(parent_node);
        if (sibling_node != i && sibling_node < q->size 
                && q->getpri(q->d[sibling_node]) == prio) {
            if (q->eqelem(q->d[sibling_node], inserted_node)) {
                pqueue_remove(q, q->d[sibling_node]);
            }
        }
    }
  
    // printf("==After dedup==\n");
    // pqueue_dump(q, stdout, q->prt);
}

static size_t
bubble_up(pqueue_t *q, size_t i)
{
    size_t parent_node;
    void *moving_node = q->d[i];
    pqueue_pri_t moving_pri = q->getpri(moving_node);

    for (parent_node = parent(i);
         ((i > 1) && q->cmppri(q->getpri(q->d[parent_node]), moving_pri));
         i = parent_node, parent_node = parent(i))
    {
        q->d[i] = q->d[parent_node];
        q->setpos(q->d[i], i);
    }

    q->d[i] = moving_node;
    q->setpos(moving_node, i);
    return i;
}

static void
percolate_down(pqueue_t *q, size_t i)
{
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


int
pqueue_insert(pqueue_t *q, void *d)
{
    void *tmp;
    size_t i;
    size_t newsize;

    if (!q) return 1;

    /* allocate more memory if necessary */
    if (q->size >= q->avail) {
        newsize = q->size + q->step;
        if (!(tmp = realloc(q->d, sizeof(void *) * newsize)))
            return 1;
        q->d = tmp;
        q->avail = newsize;
    }
    /* insert item and remove potential duplicate */
    i = q->size++;
    q->d[i] = d;
    dedup(q, bubble_up(q, i));
    return 0;
}

int
pqueue_remove(pqueue_t *q, void *d)
{
    size_t posn = q->getpos(d);
    q->d[posn] = q->d[--q->size];
    if (q->cmppri(q->getpri(d), q->getpri(q->d[posn])))
        bubble_up(q, posn);
    else
        percolate_down(q, posn);

    return 0;
}


void *
pqueue_pop(pqueue_t *q)
{
    void *head;
    
    if (!q || q->size == 1)
        return NULL;
        
    head = q->d[1];
    q->d[1] = q->d[--q->size];
    percolate_down(q, 1);
    
    return head;
}

void *
pqueue_find(pqueue_t *q, void *e, int pos, pqueue_pri_t max)
{
    void* rval;

    if ((pos == 1 && (!q || q->size == 1)) 
            || q->d[pos] == NULL || q->getpri(q->d[pos]) > max)
        return NULL;
    
    if (q->eqelem(q->d[pos], e) && (q->getpri(e) - q->getpri(q->d[pos]) < max)) {
        // printf("Prio of e: %lld\n", q->getpri(e));
        // printf("Prio of found: %lld\n", q->getpri(q->d[pos]));
        // printf("max: %lld", max);
        return q->d[pos];
    } else {
        rval = pqueue_find(q, e, left(pos), max);
        if (rval) 
            return rval;
        else {
            // Continue search on right branch
            return pqueue_find(q, e, right(pos), max);
        }
    }
}

void *
pqueue_peek(pqueue_t *q)
{
    void *d;
    if (!q || q->size == 1)
        return NULL;
    d = q->d[1];
    return d;
}

void
pqueue_dump(pqueue_t *q,
            FILE *out,
            pqueue_print_entry_f print)
{
    int i;

    fprintf(stdout,"posn\tleft\tright\tparent\tmaxchild\t...\n");
    for (i = 1; i < q->size ;i++) {
        fprintf(stdout,
                "%d\t%d\t%d\t%d\t%ul\t",
                i,
                left(i), right(i), parent(i),
                (unsigned int)maxchild(q, i));
        print(out, q->d[i]);
    }
}

void
pqueue_print(pqueue_t *q,
             FILE *out,
             pqueue_print_entry_f print)
{
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
		print(out, e);

    pqueue_free(dup);
}


static int
subtree_is_valid(pqueue_t *q, int pos)
{
    if (left(pos) < q->size) {
        /* has a left child */
        if (q->cmppri(q->getpri(q->d[pos]), q->getpri(q->d[left(pos)])))
            return 0;
        if (!subtree_is_valid(q, left(pos)))
            return 0;
    }
    if (right(pos) < q->size) {
        /* has a right child */
        if (q->cmppri(q->getpri(q->d[pos]), q->getpri(q->d[right(pos)])))
            return 0;
        if (!subtree_is_valid(q, right(pos)))
            return 0;
    }
    return 1;
}


int
pqueue_is_valid(pqueue_t *q)
{
    return subtree_is_valid(q, 1);
}
