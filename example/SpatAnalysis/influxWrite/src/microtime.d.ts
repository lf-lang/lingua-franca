/**
 * Timer used for precisely timing the triggering of reactions.
 * Module used to acquire time values from the platform in microsecond precision.
 * @see {@link https://www.npmjs.com/package/microtime}
 */
declare module 'microtime' {
    export default class MicroTime {
        
        /**
         * Get the current time in microseconds as an integer.
         */
        static now(): number;

        /**
         * Get the current time in seconds as a floating point number with
         * microsecond accuracy (similar to `time.time()` in Python and
         * `Time.now.to_f` in Ruby).
         */
        static nowDouble(): number;

        /**
         * Get the current time and return as a list with seconds and
         * microseconds (matching the return value of `gettimeofday(2)`).
         */
        static nowStruct(): [number, number];

    }
}



