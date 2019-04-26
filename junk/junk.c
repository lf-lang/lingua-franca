#include <stdio.h>
#include <stdlib.h>
#include "pqueue.h"

typedef int interval_t;

typedef int index_t;     // topological index

typedef int handle_t;   

typedef struct instant_t {
  int time;         // a point in time
  int microstep;    // superdense time index
  size_t pos;
} instant_t; // FIXME: maybe call it tick?

typedef struct trigger_t {
  void** reactions; // FIXME: more specific type to include argument types.
  interval_t minOffset;
  interval_t minPeriod;
} trigger_t;

typedef struct event_t {
  instant_t tag;            // tag of the event
  trigger_t* trigger;    // pointer to the trigger
  size_t pos;            // position in the priority queue 
} event_t;

/* Return the index of a reaction */
static index_t get_index(void* reaction) {
  return 0; // FIXME
}

instant_t current_time = {0,0};

static int cmp_pri(pqueue_pri_t next, pqueue_pri_t curr) {
  return (next < curr);
}

static pqueue_pri_t get_pri(void *a) {
  // stick the time and microstep together into an unsigned long long
  return ((pqueue_pri_t)(((event_t *) a)->tag.time) << 32) | (pqueue_pri_t)(((event_t *) a)->tag.microstep);
}

static void set_pri(void *a, pqueue_pri_t pri) {
  // ignore this; priorities are fixed
}

static size_t get_pos(void *a) {
  return ((event_t *) a)->pos;
}

static void set_pos(void *a, size_t pos) {
  ((event_t *) a)->pos = pos;
}

pqueue_t* eventQ;

handle_t schedule(trigger_t* trigger, interval_t delay) {
     event_t* e = malloc(sizeof(struct event_t));
     e->tag.time = current_time.time;
     e->tag.microstep = current_time.microstep+1;
     e->trigger = trigger;
     pqueue_insert(pq, eventQ);
}

reaction_t* next() {
  // wait until t >= T

  // pop all events from eventQ with timestamp equal to currentTime

  // stick them into reactionQ

  // while popping reactions from the reactionQ:
  // execute them, pop events off the eventQ and insert them into reactionQ
  
}

int main() {
  eventQ = pqueue_init(10, cmp_pri, get_pri, set_pri, get_pos, set_pos);
  trigger_t trigger = {NULL, 0, 0};
  schedule(&trigger, 0);
}
