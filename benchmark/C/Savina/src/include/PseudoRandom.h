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

/**
 * Initialize the random number generator to the specified seed.
 */
void initPseudoRandom(struct PseudoRandom* random, long seed) {
    random->mValue = seed;
}

/**
 * Return a random number between 0 and 65535
 */
long nextLong(struct PseudoRandom *random) {
    random->mValue = ((random->mValue * 1309) + 13849) & 65535;
    return random->mValue;
}

/**
 * Return a random number between 0 and 65535
 */
int nextInt(struct PseudoRandom *random) {
    return (int)nextLong(random);
}

/**
 * Return a random number between 1.0/65536
 * and 1.0.
 */
double nextDouble(struct PseudoRandom *random) {
    return 1.0 / (nextLong(random) + 1);
}

/**
 * Return a random number between 0 and exclusive_max - 1.
 */
int nextIntEMax(struct PseudoRandom *random, int exclusive_max) {
    return nextInt(random) % exclusive_max;
}

#endif // PSUEDO_RANDOM_H
