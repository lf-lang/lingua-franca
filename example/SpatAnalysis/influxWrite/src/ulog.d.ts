/**
 * Microscopically small universal logging library.
 * @see {@link https://www.npmjs.com/package/ulog}
 */
declare module 'ulog' {
    export default function ULog(module: string): ULog;
}

/**
 * Typed interface for the NPM ulog module.
 */
interface ULog {
    
    /**
     * Level of severity required for messages to be displayed.
     */
    level: number;
    
    /**
     * This logs a DEBUG message.
     * @param message 
     */
    debug(message: string): void;
    
    /**
     * Enables debug mode for the loggers listed in str.
     * @param str
     */
    enable(str: string): void;
    
    /**
     * Tests whether the logger is currently in debug mode.
     * @param module 
     */
    enabled(module: string): boolean;
    
    /**
     * Disables debug mode for all loggers.
     */
    disable():void;

    /**
     * This logs an ERROR message.
     * @param message 
     */
    error(message: string): void;
    
    /**
     * This logs an INFO message.
     * @param message 
     */
    info(message: string): void;

    /**
     * This logs a LOG message.
     * @param message 
     */
    log(message: string): void;

    /**
     * This logs a TRACE message.
     * @param message 
     */
    trace(message: string): void;

    /**
     * This logs a WARN message.
     * @param message 
     */
    warn(message: string): void;
}