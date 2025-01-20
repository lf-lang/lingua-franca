#include "tag.h"

int lf_clock_gettime(instant_t* now) {
  static instant_t clock = 0;
  *now = clock++; 
}