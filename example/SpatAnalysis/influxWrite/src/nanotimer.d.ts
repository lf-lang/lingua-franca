/**
 * Timer used for precisely timing the triggering of reactions.
 * @see {@link https://www.npmjs.com/package/nanotimer}
 */
declare module 'nanotimer' {
    export default class NanoTimer {
        constructor();
        
        /**
         * Clears current running interval.
         */
        clearInterval(): void;
        
        /**
         * Clears current running timeout.
         */
        clearTimeout(): void;

        /**
         * Returns true if the timer currently has a scheduled timeout, or false otherwise.
         */
        hasTimeout(): boolean;

        /**
         * 
         * @param task 
         * @param args 
         * @param interval 
         * @param callback 
         */
        setInterval(task: Function, args:[]|[unknown]|"", interval:string, callback?: (data: {waitTime: number}) => void):void;

        /**
         * 
         * @param task 
         * @param args 
         * @param timeout 
         * @param callback 
         */
        setTimeout(task: Function, args:[]|[unknown]|"", timeout:string, callback?: (data: {waitTime: number}) => void):void;

        /**
         * 
         * @param task 
         * @param args 
         * @param format 
         * @param callback 
         */
        time(task: Function, args:[]|[unknown]|"", format:"s"|"m"|"u"|"n", callback?: (data: {waitTime: number}) => void): void;

    }
}
