package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

abstract class StaticSchedulerBase implements StaticScheduler {

    Dag dag;
    
    public StaticSchedulerBase(Dag dag) {
        this.dag = dag;
    }

    public Dag getDag() {
        return this.dag;
    }
    
}
