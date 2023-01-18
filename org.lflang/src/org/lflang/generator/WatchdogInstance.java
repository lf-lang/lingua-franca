package org.lflang.generator;

import org.lflang.TimeValue;
import org.lflang.lf.Watchdog;

/**
 * Instance of a watchdog. Upon creation the actual delay is converted into
 * a proper time value. If a parameter is referenced, it is looked up in the
 * given (grand)parent reactor instance.
 * 
 * @author{Benjamin Asch <benjamintasch@berkeley.edu>}
 */
//FIXME: modif4watchdogs
//FIXME: functions in constructor not defined
public class WatchdogInstance {
		
	/**
	 * Create a new watchdog instance associated with the given reaction
	 * instance.
	 */
	public WatchdogInstance(Watchdog definition, ReactorInstance reactor) {
        if (definition.getTimeout() != null) {
            // WATCHDOG QUESTION
            // How does this .getTimeValue work? Where is expression coming from 
            // versus other time parameters?
            this.timeout = reactor.getTimeValue(definition.getTimeout());
        } else {
            this.timeout = TimeValue.ZERO;
        }

        this.name = definition.getName();
    }

    public String getName() {
        return this.name;
    }

    //////////////////////////////////////////////////////
    //// Public fields.

    /**
     * The timeout, L, associated with this deadline. The physical timer gets set 
     * to expire at physical time, P, equal to the current logical time, t, plus the timeout.
     * 
     * In other words, the watchdog condition is met iff P < t + L while the watchdog has not 
     * been stopped or reset.
     */
    public final TimeValue timeout;

    /**
     * The watchdog name.
     */
    public final String name;

    //////////////////////////////////////////////////////
    //// Public methods.
	
    //FIXME: unsure of use or need for watchdogs
    @Override
    public String toString() {
        return "WatchdogInstance " + timeout.toString();
    }
}
