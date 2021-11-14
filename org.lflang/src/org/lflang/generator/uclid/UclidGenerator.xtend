/* Generator for Uclid models. */

/*************
Copyright (c) 2021, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.generator

import java.nio.file.Path
import java.nio.file.Paths
import java.util.ArrayList
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.lf.Action
import org.lflang.lf.VarRef
import org.lflang.Target

import static extension org.lflang.ASTUtils.*

/**
 * Generator for Uclid models.
 * 
 * Connectivity graph needs to preserve the following info:
 * 1. Components,
 * 2. Connectivity edges (connection, action, timer),
 *   - Types of connectivity edges (logical, physical)
 *   - Delays on the edges 
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
 
class UclidGenerator extends GeneratorBase {
    
    ////////////////////////////////////////////
    //// Public variables
    
    // The output directory where the model is stored
    Path outputDir
    
    var ConnectivityGraph connectivityGraph
    
    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        super(fileConfig, errorReporter)
    }
    
    ////////////////////////////////////////////
    //// Private variables
    
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
        IGeneratorContext context) {
            
        // TODO: remove this.
        super.doGenerate(resource, fsa, context)
        
        // Traverse the instantiation tree for the connectivity information
        buildConnectivityGraph()
        
        // Create the "src-gen" directory if it doesn't yet exist.
        var dir = fileConfig.getSrcGenPath.toFile
        if (!dir.exists()) dir.mkdirs()
        
        // Create the "model" directory if it doesn't yet exist.
        dir = fileConfig.getSrcGenPath.resolve("model").toFile
        if (!dir.exists()) dir.mkdirs()
        outputDir = Paths.get(dir.getAbsolutePath)
        
        generateModel()
    }
    
    def buildConnectivityGraph() {   
        // FIXME: Does this handle the federated reactor in case of federated execution?    
        for (federate : federates) {
            // Build the instantiation tree if a main reactor is present.
            if (this.mainDef !== null) {
                // generateReactorFederated(this.mainDef.reactorClass, federate)
                if (this.main === null) {
                    // Recursively build instances. This is done once because
                    // it is the same for all federates.
                    this.main = new ReactorInstance(
                        mainDef.reactorClass.toDefinition,
                        this.errorReporter
                    )
                }   
            }
        }  
        
        this.connectivityGraph = new ConnectivityGraph(this.main)
    }
    
    protected def generateModel() {     
        // Generate main.ucl and print to file
        code = new StringBuilder()
        var filename = outputDir.resolve("main.ucl").toString
        generateMain()
        writeSourceCodeToFile(getCode.getBytes, filename)
        
        // Generate run.sh
        code = new StringBuilder()
        filename = outputDir.resolve("run.sh").toString
        generateRunScript()
        writeSourceCodeToFile(getCode.getBytes, filename)
    }
    
    protected def generateMain() {
        pr('''
        /*******************************
         * Auto-generated UCLID5 model *
         ******************************/
         
        ''')
        
        // Start the main module
        pr('''
        module main {
         
        ''')
        indent()
        
        // [static] definition of time and timing-related operations
        pr('''
        /*******************************
         * Time and Related Operations *
         ******************************/
        type timestamp_t = integer; // Unit is nanoseconds
        type microstep_t = integer;
        type tag_t = {
            timestamp_t,
            microstep_t
        };
        // FIXME: in LF, the interval is an integer.
        type interval_t  = tag_t;
        
        // Projection macros
        define pi1(t : tag_t)   : timestamp_t   = t._1;         // Get timestamp from tag
        define pi2(t : tag_t)   : microstep_t   = t._2;         // Get microstep from tag
        
        // Interval constructor
        define zero() : interval_t
        = {0, 0};
        define startup() : interval_t
        = zero();
        define mstep() : interval_t
        = {0, 1};
        define nsec(t : integer) : interval_t
        = {t, 0};
        define usec(t : integer) : interval_t
        = {t * 1000, 0};
        define msec(t : integer) : interval_t
        = {t * 1000000, 0};
        define sec(t : integer) : interval_t
        = {t * 1000000000, 0};
        define inf() : interval_t
        = {-1, 0};
        
        // Helper function
        define isInf(i : interval_t) : boolean
        = pi1(i) < 0;
        
        // Tag comparison
        define tag_later(t1 : tag_t, t2 : tag_t) : boolean
        = pi1(t1) > pi1(t2)
            || (pi1(t1) == pi1(t2) && pi2(t1) > pi2(t2))
            || (isInf(t1) && !isInf(t2));
        
        define tag_same(t1 : tag_t, t2 : tag_t) : boolean
        = t1 == t2;
        
        define tag_earlier(t1 : tag_t, t2 : tag_t) : boolean
        = pi1(t1) < pi1(t2)
            || (pi1(t1) == pi1(t2) && pi2(t1) < pi2(t2))
            || (!isInf(t1) && isInf(t2));
        
        // Tag algebra
        define tag_schedule(t : tag_t, i : interval_t) : tag_t
        = if (!isInf(t) && pi1(i) == 0 && !isInf(i))
            then { pi1(t), pi2(t) + 1 } // microstep delay
            else ( if (!isInf(t) && pi1(i) > 0 && !isInf(i))
                then { pi1(t) + pi1(i), 0 }
                else inf());
        
        define tag_delay(t : tag_t, i : interval_t) : tag_t
        = if (!isInf(t) && !isInf(i))
            then { pi1(t) + pi1(i), pi2(t) + pi2(i) }
            else inf();
        
        // Only consider timestamp for now.
        define tag_diff(t1, t2: tag_t) : interval_t
        = if (!isInf(t1) && !isInf(t2))
            then { pi1(t1) - pi1(t2), pi2(t1) - pi2(t2) }
            else inf();
         
        ''')
        
        // [dynamic] Encode the components and
        // the logical delays present in a reactor system.
        pr('''
        /**********************************
         * Reactions & connectivity graph *
         *********************************/
        
        //////////////////////////
        // Application Specific
        ''')
        
        // TODO: generate custom code.
        pr('''
        // Element IDs
        type id_t = enum {  
            NULL,                       // NULL 
            A1, B1, C1,                 // Reactions
            STARTUP,                    // Actions
            A_out, B_in, B_out, C_in,   // Ports 
            B_ramp_exists, C_s          // Variables
        };
        //////////////////////////
         
        ''')
        
        pr('''
        /*****************
         * Trace Element *
         ****************/
        type element_t = { id_t, tag_t };
        
        // Projection macros
        define id(e : element_t) : id_t     = e._1;
        define g(e : element_t) : tag_t     = e._2;
        
        define isNULL(e : element_t) : boolean = id(e) == NULL;
         
        ''')
        
        pr('''
        /********************
         * Trace Definition *
         *******************/
        const START : integer = 0;
        const END : integer = 9;

        define in_range(num : integer) : boolean
        = num >= START && num <= END;

        type step_t = integer;
        type trace_t = { 
            element_t,
            element_t,
            element_t,
            element_t,
            element_t,
            element_t,
            element_t,
            element_t,
            element_t,
            element_t
        };

        define get(tr : trace_t, i : step_t) : element_t 
        = if (i == 0) then tr._1 else (
            if (i == 1) then tr._2 else (
                if (i == 2) then tr._3 else (
                    if (i == 3) then tr._4 else (
                        if (i == 4) then tr._5 else (
                            if (i == 5) then tr._6 else (
                                if (i == 6) then tr._7 else (
                                    if (i == 7) then tr._8 else (
                                        if (i == 8) then tr._9 else (
                                            if (i == 9) then tr._10 else (
                                                { NULL, inf() }
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                )
            )
        );

        define in_trace(_tr : trace_t, _e : element_t) : boolean
        = (exists (j : step_t) :: in_range(j) && get(_tr, j) == _e);
         
        ''')
        
        // Mark the start of the trace
        pr('''
        // Mark the start of the trace.
        var start : timestamp_t;
        assume(start == pi1(zero()));
         
        ''')
        
        // Instantiate the trace        
        pr('''
        // Declare the trace
        // The counterexample reflects a particular frame of reference.
        var trace : trace_t;
        
        // Helper macro that returns an element based on index
        define elem(i : integer) : element_t
        = get(trace, i);
         
        ''')
        
        // Encode the topology
        pr_topology()
        
        // Encode the HB relation
        pr_happened_before()
        
        // Encode topological abstraction
        pr_topo_abstraction()
        
        // Encode reactor semantics
        pr_reactor_semantics()
        
        // Encode topological axioms
        pr_topo_axioms()
        
        // [placeholder] Add user-defined properties
        pr('''
        // [placeholder] Add user-defined properties here.
         
        //////////////////////////////////////////////////
         
        ''')
        
        pr('''
        control {
            v = unroll(0);
            check;
            print_results;
            v.print_cex;
        }
         
        ''')
        
        unindent()
        pr('}')
    }
    
    def pr_topo_axioms() {
        pr('''
        /*******************************
         * Application-specific axioms *
         *******************************/
         
         
        ''')
    }
    
    // Encode reactor semantics
    def pr_reactor_semantics() {
        pr('''
        /*********************
         * Reactor Semantics *
         *********************/
        
        // [Important] The trace should respect the HB relation.
        // In this case, there is no constraint on i, j when hb
        // fails to establish on both direction (def. of concurrent).
        axiom(forall (i, j : integer) :: (in_range(i) && in_range(j))
            ==> (_hb(elem(i), elem(j)) ==> i < j));
        
        // All microsteps are positive
        axiom(forall (i : integer) :: in_range(i)
            ==> pi2(g(elem(i))) >= 0);

        // The same event can only trigger once in a logical instant
        axiom(forall (i, j : integer) :: (in_range(i) && in_range(j))
            ==> ((id(elem(i)) == id(elem(j)) && i != j)
                ==> !tag_same(g(elem(i)), g(elem(j)))));

        // NULL events should appear in the suffix
        axiom(forall (j : integer) :: in_range(j) ==> (
            (id(elem(j)) != NULL) ==> (forall (i : integer) :: in_range(i) ==> 
            ((i < j) ==> id(elem(i)) != NULL)
        )));
        
        // All tags should be positive
        axiom(forall (i : integer) :: in_range(i) ==> (
            pi1(g(elem(i))) >= 0
        ));

        // Begin the frame at the start time specified.
        axiom(forall (i : integer) :: tag_same(g(elem(i)), {start, 0})
            || tag_later(g(elem(i)), {start, 0}));
         
        ''')
    }
    
    // Encode topological abstraction to enable easy code generation.
    def pr_topo_abstraction() {
        pr('''
        /***************************
         * Topological Abstraction *
         ***************************/
        define is_multiple_of(a, b : integer) : boolean
        = exists (c : integer) :: b * c == a;

        define is_closest_starting_point(t : tag_t, period : integer, offset : integer) : boolean
        = (exists (c : integer) :: (period * c) + offset == pi1(t)
            // Tick at the next valid instant.
            && (period * (c - 1) + offset) < start)     
            // Timer always has mstep of 0.
            && pi2(t) == 0;                           

        // first & last in trace
        define first(e : element_t) : boolean
        = !(exists (i : integer) :: in_range(i) && id(elem(i)) == id(e) && tag_earlier(g(elem(i)), g(e))); 

        define last(e : element_t) : boolean
        = !(exists (i : integer) :: in_range(i) && id(elem(i)) == id(e) && tag_later(g(elem(i)), g(e))); 

        define is_triggered_by_startup(_id : id_t) : boolean
        = // If startup is within frame, put the events in the trace.
        ((start == 0) ==> (exists (i : integer) :: in_range(i)
            && id(elem(i)) == _id && tag_same(g(elem(i)), startup())))
        // Can only appear once.
        && !(exists (j : integer) :: in_range(j) && id(elem(j)) == _id
            && !tag_same(g(elem(j)), startup()));

        // Can directly use index as HB since this only applies to events
        // on the same federate.
        define _is_latest_invocation_in_same_fed_wrt_(a, b : integer) : boolean
        = !(exists (c : integer) :: in_range(c) 
            && id(elem(c)) == id(elem(a)) && a < c && c < b);

        define is_triggered_by_timer(_id : id_t) : boolean
        =   // 1. If the initial event is within frame, show it.
            (exists (i : integer) :: in_range(i)
            && id(elem(i)) == _id
            && is_closest_starting_point(g(elem(i)), pi1(timer_period(_id)),
                pi1(timer_offset(_id))))
            // 2. The SPACING between two consecutive timers is the period.
            && (forall (i, j : integer) :: (in_range(i) && in_range(j) && i < j
                && id(elem(i)) == _id && id(elem(j)) == _id
                // ...and there does not exist a 3rd invocation in between
                && !(exists (k : integer) :: id(elem(k)) == _id && i < k && k < j))
                    ==> g(elem(j)) == tag_schedule(g(elem(i)), timer_period(_id)))
            // 3. There does not exist other events in the same federate that 
            // differ by the last timer invocation by g(last_timer) + period.
            // In other words, this axiom ensures a timer fires when it needs to.
            //
            // a := index of the offending event that occupy the spot of a timer tick.
            // b := index of non-timer event on the same federate
            // both in_range's are needed due to !(exists), which turns into a forall.
            && !(exists (b, a : integer) :: in_range(a) && in_range(b)
                && id(elem(b)) != _id
                && _id_same_fed(elem(b), {_id, zero()})
                && id(elem(a)) == _id
                && (_is_latest_invocation_in_same_fed_wrt_(a, b)
                    && tag_later(g(elem(b)), tag_schedule(g(elem(a)), timer_period(_id))) ));
        
        // This includes the possibility that upstream does NOT output.
        define is_triggered_by(downstream, upstream : id_t, delay : interval_t) : boolean
        = (forall (i : integer) :: in_range(i) ==>
            id(elem(i)) == downstream ==> (exists (j : integer) :: in_range(j)
                && id(elem(j)) == upstream 
                && g(elem(i)) == tag_schedule(g(elem(j)), delay))
        );

        // This macro ensures that the upstream MUST output.
        define is_definitely_triggered_by(downstream, upstream : id_t,
            delay : interval_t) : boolean
        = (forall (i : integer) :: in_range(i) ==>
            id(elem(i)) == downstream ==> (exists (j : integer) :: in_range(j)
                && id(elem(j)) == upstream 
                && g(elem(i)) == tag_schedule(g(elem(j)), delay))
        ) && 
        (forall (j : integer) :: in_range(j) ==>
            id(elem(j)) == upstream ==> (exists (i : integer) :: in_range(i)
                && id(elem(i)) == downstream 
                && g(elem(i)) == tag_schedule(g(elem(j)), delay))
        );

        define is_in_trace(_id : id_t) : boolean
        = (exists (i : integer) :: in_range(i) && id(elem(i)) == _id);
         
        ''')
    }
    
    // Generate macros that encode dynamic HB relation.
    def pr_happened_before() {
        pr('''
        // Return true if e1 happened before e2 in the same federate,
        // by comparing the tags and the priorities of the two events.
        //
        // TODO: account for nested reactors in federates.
        define _hb_same_fed(_e1, _e2 : element_t) : boolean
        = tag_earlier(g(_e1), g(_e2)) ||
            (tag_same(g(_e1), g(_e2)) && priority(id(_e1)) < priority(id(_e2)));
        
        // Return true if e1 happened before e2 in different federates
        // and e1 has a connection to e2. This check uses the time tags
        // and the logical delay in the connection.
        //
        // FIXME: account for physical connections.
        define _hb_diff_fed(_e1, _e2 : element_t) : boolean
        = tag_earlier(tag_delay(g(_e1), connection_delay(id(_e1), id(_e2))), g(_e2))
            || tag_same(tag_delay(g(_e1), connection_delay(id(_e1), id(_e2))), g(_e2));
        
        // Check the happened-before relation between two immediate events.
        define _hb(_e1, _e2 : element_t) : boolean
        =   // If two events belong to the same federate,
            // determine hb via tags.
            // This is bi-directional.
            (_id_same_fed(_e1, _e2) && _hb_same_fed(_e1, _e2))
            // If two events belong to different federates,
            // check if a connection is present.
            // This is uni-directional.
            || (_id_diff_fed(_e1, _e2) && _hb_diff_fed(_e1, _e2));
        
        // HB path with 1 edge
        define hb_1(e1, e2 : element_t) : boolean
        // Link e1, e2 to the start and end of a path.
        = exists (a, b : integer) :: in_range(a) && in_range(b) 
            // Check if a path between e1 and e2 exists.
            && elem(a) == e1 && elem(b) == e2
            && _hb(elem(a), elem(b))
            && a < b; 
        
        // HB path with 2 edges
        define hb_2(e1, e2 : element_t) : boolean
        // Link e1, e2 to the start and end of a path.
        = exists (a, b, c : integer) :: elem(a) == e1 && elem(c) == e2 
            // Check if a path between e1 and e2 exists.
            && _hb(elem(a), elem(b)) && _hb(elem(b), elem(c))
            && a < b && b < c;
        
        // HB path with 3 edges
        define hb_3(e1, e2 : element_t) : boolean
        // Link e1, e2 to the start and end of a path.
        = exists (a, b, c, d : integer) :: elem(a) == e1 && elem(d) == e2 
            // Check if a path between e1 and e2 exists.
            && _hb(elem(a), elem(b)) && _hb(elem(b), elem(c)) && _hb(elem(c), elem(d))
            && a < b && b < c && c < d;
        
        // Transitive "happened-before" definition
        define hb(e1, e2 : element_t) : boolean
        = hb_1(e1, e2) || hb_2(e1, e2) || hb_3(e1, e2);
         
        ''')
    }
    
    // Generate macros that contain information about the system topology.
    def pr_topology() {
        pr('''
        /************
         * Topology *
         ************/
         
        ''')
    }
    
    protected def generateRunScript() {
        pr('''
        #!/bin/bash
        set -e # Terminate on error
        
        echo '*** Setting up smt directory'
        rm -rf ./smt/ && mkdir -p smt
        
        echo '*** Generating SMT files from UCLID5'
        uclid -g "smt/output" common.ucl main.ucl
        
        echo '*** Append (get-model) to each file'
        ls smt | xargs -I {} bash -c 'echo "(get-model)" >> smt/{}'
        
        echo '*** Running Z3'
        ls smt | xargs -I {} bash -c 'echo "Checking {}" && z3 -T:120 ./smt/{}'
        ''')
    }
    
    /////////////////////////////////////////////////
    //// Helper functions
    
    protected def ArrayList<String> populateIds() {
        var triggerIDs = new ArrayList<String>();
        for (r : reactors) {
            for (rxn : r.getReactions()) {
                for (t : rxn.getTriggers) {
                    if (t.isStartup()) {
                        triggerIDs.add(r.name + '_' + 'startup')
                    }
                    else if (t instanceof VarRef) {
                        triggerIDs.add(r.name + '_' + t.variable.name)
                    }
                }
            }
        }
        return triggerIDs
    }

    override getTarget() {
        return Target.C // FIXME: How to make this target independent?
    }
    
    /////////////////////////////////////////////////
    //// Functions from generatorBase
    
    override generateDelayBody(Action action, VarRef port) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override generateForwardBody(Action action, VarRef port) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override generateDelayGeneric() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override supportsGenerics() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override getTargetTimeType() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override getTargetTagType() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override getTargetUndefinedType() {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override getTargetFixedSizeListType(String baseType, int size) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }

    override getTargetVariableSizeListType(String baseType) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
 
}