/**
 * Simple deterministic pseudo random number generator for
 * some benchmarks.
 *
 * Original author: Hannes Klein
 * Adapted to C by: Soroush Bateni
 */
#ifndef PSUEDO_RANDOM_H
#define PSUEDO_RANDOM_H

struct pseudo_random {
	long mValue;
};

void init_pseudo_random(struct pseudo_random* random ,long mValue) {
    random->mValue = mValue;
}

long next_long(struct pseudo_random *random) {
    random->mValue = ((random->mValue * 1309) + 13849) & 65535;
    return random->mValue;
}

int next_int(struct pseudo_random *random) {
    return (int) next_long(random);
}

double next_double(struct pseudo_random *random) {
    return 1.0 / (next_long(random) + 1);
}

int next_int_exclusive_max(struct pseudo_random *random, int exclusive_max) {
    return next_int(random) % exclusive_max;
}

#endif // PSUEDO_RANDOM_H
