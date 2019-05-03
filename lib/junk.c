#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include "pqueue.h"

#define CLOCKID CLOCK_REALTIME
#define errExit(msg) do { perror(msg); exit(EXIT_FAILURE); } while (0)
#define SIG SIGRTMIN

_Bool check = 0;

typedef int interval_t;

typedef int index_t;     // topological index

typedef int handle_t;   

typedef struct instant_t {
  int time;         // a point in time
  int microstep;    // superdense time index
} instant_t;

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

static void handler(int sig, siginfo_t *si, void *uc) {
  /* Note: calling printf() from a signal handler is not safe
    (and should not be done in production programs), since
    printf() is not async-signal-safe; see signal-safety(7).
    Nevertheless, we use printf() here as a simple way of
    showing that the handler was called. */

  printf("Caught signal %d\n", sig);
  check = 1;
  //signal(sig, SIG_IGN); // This is to stop the timer
}

pqueue_t* eventQ;

handle_t schedule(trigger_t* trigger, interval_t delay) {
     event_t* e = malloc(sizeof(struct event_t));
     e->tag.time = current_time.time;
     e->tag.microstep = current_time.microstep+1;
     e->trigger = trigger;
     pqueue_insert(eventQ, e);
}

typedef struct reaction_t {
  void* func;
  index_t index;
  // add uses, produces, etc.
  size_t pos;
} reaction_t;

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

  timer_t timerid;
  struct sigevent sev;
  struct itimerspec its;
  long long freq_nanosecs;
  sigset_t mask;
  struct sigaction sa;

   /* Establish handler for timer signal */
  printf("Establishing handler for signal %d\n", SIG);
  sa.sa_flags = SA_SIGINFO;
  sa.sa_sigaction = handler;
  sigemptyset(&sa.sa_mask);
  if (sigaction(SIG, &sa, NULL) == -1)
    errExit("sigaction");

  /* Create the timer */
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SIG;
  sev.sigev_value.sival_ptr = &timerid;
  if (timer_create(CLOCKID, &sev, &timerid) == -1)
    errExit("timer_create");
  

  /* Start the timer */

  freq_nanosecs = 1000000000;
  its.it_value.tv_sec = freq_nanosecs / 1000000000;
  its.it_value.tv_nsec = freq_nanosecs % 1000000000;
  its.it_interval.tv_sec = its.it_value.tv_sec;
  its.it_interval.tv_nsec = its.it_value.tv_nsec;

  if (timer_settime(timerid, 0, &its, NULL) == -1)
      errExit("timer_settime");

  while(1) {
    if(check == 1) {
      printf("Woken up.\n");
      check = 0;
    }  
  }
   

}
