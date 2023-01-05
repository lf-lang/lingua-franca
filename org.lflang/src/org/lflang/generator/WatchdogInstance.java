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
	public WatchdogInstance(Watchdog definition, ReactionInstance reaction) {
        if (definition.getTimeout() != null) {
            this.timeout = reaction.parent.getTimeValue(definition.getTimeout());
        } else {
            this.timeout = TimeValue.ZERO;
        }

        if (definition.getCode() != null) {
            this.watchdogHandler = definition.getCode();
        } else {
            this.watchdogHandler = new CodeBuilder();
        }

        this.name = definition.getName();
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
     * The code body for the function that is executed upon calling the watchdog.
     */
    public final CodeBuilder watchdogHandler;

    //////////////////////////////////////////////////////
    //// Public methods.
	
    //FIXME: unsure of use or need for watchdogs
    @Override
    public String toString() {
        return "WatchdogInstance " + timeout.toString();
    }
}
