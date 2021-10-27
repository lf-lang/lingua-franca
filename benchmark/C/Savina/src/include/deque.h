/**
 * Simple implementation for deque in C
 * 
 * Supported Operations: deque_push_front, deque_push_back, deque_pop_front, deque_pop_back, deque_peek_front, 
 * deque_peek_back, deque_is_empty, deque_get_size.
 * 
 * Value stored in the deque is of type void* to make the use generic. Memory allocation/free is handled internally. Deque initialization requires
 * information about the size of value to be stored.
 * 
 * author: Arthur Deng
 */

#ifndef DEQUE_H
#define DEQUE_H

typedef struct node {
    struct node *next;
    struct node *prev;
    void* value;
} node;

typedef struct deque {
    struct node* front;
    struct node* back;
    size_t size;
    int length;
} deque;

/*
* Function:  deque_initialize
* --------------------
* allocates memory for and intializes the deque.
* size: size of each data item
*  returns: pointer to the deque
*/
deque* deque_initialize(size_t size) {
    // allocate space for deque
    deque *p = (deque *) malloc (sizeof(deque));
    if (p != NULL) {
        p->front = NULL;
        p->back = NULL;
        p->length = 0;
        p->size = size;
    }
    return p;
}

/*
* Function:  deque_is_empty
* --------------------
*  d: pointer to the deque
*  returns: 1 if deque is empty, 0 if deque is not empty
*/
int deque_is_empty(struct deque* d) {
    if (d==NULL) {
        return 1;
    } else if (d->length != 0) {
        return 0;
    }
    return 1;
}

/*
* Function:  _deque_create_node
* --------------------
* allocates memory for new node in deque. Used internally by deque_push_front
* and deque_push_back. Memory is freed when node is popped using deque_pop_back
* and deque_pop_front.
*  val: value to be stored in the node
*  returns: pointer to the node
*/
node* _deque_create_node(void* val) {
    node *new_node = (struct node *) malloc(sizeof(struct node));
    new_node->value = val;
    new_node->next = NULL;
    new_node->prev = NULL;
    return new_node;
}

/*
* Function:  deque_push_front
* --------------------
* Appends val to the front of the deque.
*  d: pointer to the deque
*  val: value to be added into the deque
*  returns: None
*/
void deque_push_front(struct deque* d, void* val) {
	// allocate space on the heap to store val
	void *val_heap = malloc(d->size);
	memcpy(val_heap, val, d->size);
    node *n = _deque_create_node(val_heap);
    if (d->back == NULL) {
        d->back = d->front = n;
    } else {
        d->front->prev = n;
        n->next = d->front;
        d->front = d->front->prev;
    }
    d->length++;
}

/*
* Function:  deque_push_back
* --------------------
* Appends val to the back of the deque.
*  d: pointer to the deque
*  val: value to be added into the deque
*  returns: None
*/
void deque_push_back(struct deque* d, void* val) {
	// allocate space on the heap to store val
	void *val_heap = malloc(d->size);
	memcpy(val_heap, val, d->size);
    node *n = _deque_create_node(val_heap);
    if (d->back == NULL) {
        d->back = d->front = n;
    } else {
        d->back->next = n;
        n->prev = d->back;
        d->back = d->back->next;
    }
    d->length++;
}

/*
* Function:  deque_pop_front
* --------------------
* Pops the front node from the deque and return its value. 
*  d: pointer to the deque
*  returns: value popped from the front
*/
void* deque_pop_front(struct deque* d) {
    if (d==NULL || d->front == NULL) {
        fprintf(stderr, "Error: popping from empty deque\n");
        return 0;
    }
    
    void* value = d->front->value;
    struct node *temp = d->front; // temporary pointer for freeing up memory
    
    if (d->front == d->back) { 
        // popping last element in deque
        d->front = d->back = NULL;
    } else {
        d->front = d->front->next;
    }
    d->length--; // decrement size
    free(temp->value); // free memory for value
    free(temp); // free memory for popped node
    return value;
}

/*
* Function:  deque_pop_back
* --------------------
* Pops the back node from the deque and return its value. 
*  d: pointer to the deque
*  returns: value popped from the back
*/
void* deque_pop_back(struct deque* d) {
    if (d==NULL || d->back == NULL) {
        fprintf(stderr, "Error: popping from empty deque\n");
        return 0;
    }
    
    void* value = d->back->value;
    struct node *temp = d->back; // temporary pointer for freeing up memory
    if (d->front == d->back) { 
        // popping last element in deque
        d->front = d->back = NULL;
    } else {
        d->back = d->back->prev;
    }
    d->length--; // decrement size
    free(temp->value); // free memory for value
    free(temp); // free memory for popped node
    return value;
}

/*
* Function:  deque_peek_front
* --------------------
* Returns value of the front node without modifying the deque.
*  d: pointer to the deque
*  returns: value of the front node
*/
void* deque_peek_front(struct deque* d) {
    if (d == NULL || d->front == NULL) {
        fprintf(stderr, "Error: peeking empty deque");
        return 0;
    }
    return d->front->value;
}

/*
* Function:  deque_peek_back
* --------------------
* Returns value of the back node without modifying the deque.
*  d: pointer to the deque
*  returns: value of the back node
*/
void* deque_peek_back(struct deque* d) {
    if (d == NULL || d->back == NULL) {
        fprintf(stderr, "Error: peeking empty deque");
        return 0;
    }
    return d->back->value;
}

/*
* Function:  deque_get_size
* --------------------
* Returns number of elements in the deque.
*  d: pointer to the deque
*  returns: number of elements in the deque (int).
*/
int deque_get_size(struct deque* d) {
    if (d == NULL) {
        fprintf(stderr, "Cannot get size of non-existant deque.");
    }
    return d->length;
}
#endif
