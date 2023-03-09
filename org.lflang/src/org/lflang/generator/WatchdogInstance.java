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

        this.name = definition.getName().toString();
        this.definition = definition;
        this.reactor = reactor;
    }

    public String getName() {
        return this.name;
    }

    public Watchdog getDefinition() {
        return this.definition;
    }

    public TimeValue getTimeout() {
        return (TimeValue) this.timeout;
    }

    public ReactorInstance getReactor() {
        return this.reactor;
    }

    //////////////////////////////////////////////////////
    //// Public fields.


    public final TimeValue timeout;

    /**
     * The watchdog name.
     */
    public final String name;

    public final Watchdog definition;

    public final ReactorInstance reactor;

    //////////////////////////////////////////////////////
    //// Public methods.
	
    //FIXME: unsure of use or need for watchdogs
    @Override
    public String toString() {
        return "WatchdogInstance " + timeout.toString();
    }
}
