package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

abstract class StaticSchedulerBase implements StaticScheduler {

    Dag dag;
    
    public StaticSchedulerBase(Dag dagRaw) {
        this.dag = dagRaw;
    }

    public Dag getDag() {
        return this.dag;
    }
    
}
