package org.lflang.target;

public class LoggingConfigurator {

    /**
     * Log levels in descending order of severity.
     *
     * @author Marten Lohstroh
     */
    public enum LogLevel {
        ERROR,
        WARN,
        INFO,
        LOG,
        DEBUG;

        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }
}
