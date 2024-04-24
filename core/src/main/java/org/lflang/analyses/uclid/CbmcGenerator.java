package org.lflang.analyses.uclid;

import java.util.List;

import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactionInstance.Runtime;
import org.lflang.generator.TargetTypes;
import org.lflang.target.Target;

/**
 * A generator that produces C files for CBMC
 * 
 * Here is an example of a generated file:

// LF input/output ports
typedef struct port_t {
    int is_present;
    int value;
} port_t;

// Maps to LF logical actions
typedef struct trigger_t {
    int is_present;
} trigger_t;

void lf_set(port_t *p, int v) {
    p->is_present = 1;
    p->value = v;
}

// Does not handle non-zero delay for now
void lf_schedule(trigger_t *t, int additional_delay) {
    if (additional_delay == 0)
        t->is_present = 1;
}

typedef struct traffic_light_self_t {
    int _mode;
    int count;
} traffic_light_self_t;

// The user needs to provide a JSON file telling the interface
// the names and types of the function inputs and outputs.
traffic_light_self_t *self;
traffic_light_self_t *init_self;
trigger_t *resetCount;
port_t *sigG;
port_t *sigY;
port_t *sigR;
port_t *pedestrian;

void traffic_light_reaction_2() {
    // RED
    if (self->_mode == 0) {
        if (self->count >= 60) {
            lf_set(sigG, 1);
            lf_schedule(resetCount, 0);
        } else {
            self->count += 1;
        }
    }
    // GREEN
    else if (self->_mode == 1) {
        if (pedestrian->is_present) {
            if (self->count >= 60) {
                lf_set(sigY, 1);
                lf_schedule(resetCount, 0);
            } else {
                self->count += 1;
            }
        } else {
            self->count += 1;
        }
    }
    // YELLOW
    else if (self->_mode == 2) {
        if (self->count >= 5) {
            lf_set(sigR, 1);
            lf_schedule(resetCount, 0);
        } else {
            self->count += 1;
        }
    }
    // PENDING
    else {
        if (self->count >= 60) {
            lf_set(sigY, 1);
            lf_schedule(resetCount, 0);
        } else {
            self->count += 1;
        }
    }
}

// nondet functions
traffic_light_self_t nondet_traffic_light_self_t();
port_t nondet_port_t();
trigger_t nondet_trigger_t();

int main() {
    // malloc inputs
    init_self = malloc(sizeof(traffic_light_self_t));
    pedestrian = malloc(sizeof(port_t));
    // malloc outputs
    self = malloc(sizeof(traffic_light_self_t));
    sigG = malloc(sizeof(port_t));
    sigY = malloc(sizeof(port_t));
    sigR = malloc(sizeof(port_t));
    resetCount = malloc(sizeof(trigger_t));
    // initialize with nondet values
    *init_self = nondet_traffic_light_self_t();
    *self = nondet_traffic_light_self_t();
    *pedestrian = nondet_port_t();
    *sigG = nondet_port_t();
    *sigY = nondet_port_t();
    *sigR = nondet_port_t();
    *resetCount = nondet_trigger_t();

    self->_mode = init_self->_mode;
    self->count = init_self->count;

    __CPROVER_assume(PRECONDITION);
    traffic_light_reaction_2();
    assert(POSTCONDITION);
}

 */
public class CbmcGenerator {

    public CbmcGenerator(List<Runtime> reactionInstances) {
        //TODO Auto-generated constructor stub
    }

    public void doGenerate() {
        System.out.println("*** In CBMC doGenerate!");
    }
}
