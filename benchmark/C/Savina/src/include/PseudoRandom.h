/**
 * Simple deterministic pseudo random number generator for
 * some benchmarks.
 *
 * Original author: Hannes Klein
 * Adapted to C by: Soroush Bateni
 */
#ifndef PSUEDO_RANDOM_H
#define PSUEDO_RANDOM_H

typedef struct PseudoRandom {
	long mValue;
} PseudoRandom;

void initPseudoRandom(struct PseudoRandom* random ,long mValue) {
    random->mValue = mValue;
}

long nextLong(struct PseudoRandom *random) {
    random->mValue = ((random->mValue * 1309) + 13849) & 65535;
    return random->mValue;
}

int nextInt(struct PseudoRandom *random) {
    return (int)nextLong(random);
}

double nextDouble(struct PseudoRandom *random) {
    return 1.0 / (nextLong(random) + 1);
}

int nextIntEMax(struct PseudoRandom *random, int exclusive_max) {
    return nextInt(random) % exclusive_max;
}

#endif // PSUEDO_RANDOM_H
