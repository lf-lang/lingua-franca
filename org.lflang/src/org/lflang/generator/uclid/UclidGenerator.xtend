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
import java.util.List
import java.util.Map
import java.util.Set
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.ConnectivityInfo
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.generator.PortInstance
import org.lflang.generator.JavaGeneratorUtils
import org.lflang.lf.Action
import org.lflang.lf.VarRef
import org.lflang.lf.StateVar
import org.lflang.Target

import static extension org.lflang.ASTUtils.*

/**
 * Generator for Uclid models.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
class UclidGenerator extends GeneratorBase {
    
    ////////////////////////////////////////////
    //// Public variables
    
    // The output directory where the model is stored
    Path outputDir
    
    // The reaction graph upon which the connectivity graph is built
    var ReactionInstanceGraph reactionGraph

    // The connectivity graph that contains runtime connections
    // and a set of port runtime instances
    var ConnectivityGraph connectivityGraph

    // Data structures storing info about the runtime topology
    var Set<ReactionInstance>                   reactions
    var Set<PortInstance>                       ports
    var List<Pair<ReactorInstance, StateVar>>   stateVars
        = new ArrayList<Pair<ReactorInstance, StateVar>>()

    // K
    int k = 2 // FIXME: pass K in from target property
    
    // Constructor
    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        super(fileConfig, errorReporter)
    }
    
    ////////////////////////////////////////////
    //// Private variables
    
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
        IGeneratorContext context) {
            
        // TODO: remove this.
        super.doGenerate(resource, fsa, context)
        
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

        // Build reaction instance graph and connectivity graph
        populateGraphsAndLists()
        
        // Create the "src-gen" directory if it doesn't yet exist.
        var dir = fileConfig.getSrcGenPath.toFile
        if (!dir.exists()) dir.mkdirs()
        
        // Create the "model" directory if it doesn't yet exist.
        dir = fileConfig.getSrcGenPath.resolve("model").toFile
        if (!dir.exists()) dir.mkdirs()
        outputDir = Paths.get(dir.getAbsolutePath)
        
        // Generate the Uclid model.
        generateModel()
    }

    /**
     * @brief Recursively add state variables to the stateVars list.
     * @param reactor A reactor instance from which the search begins.
     */
    private def void populateStateVars(ReactorInstance reactor) {
        var defn = reactor.reactorDefinition
        var stateVars = defn.getStateVars

        // Prefix state vars with reactor name
        // and add them to the list
        for (s : stateVars) {
            this.stateVars.add(new Pair(reactor, s))
        }

        // Populate state variables recursively
        for (child : reactor.children) {
            populateStateVars(child)
        }
    }

    /**
     * Populate the data structures.
     */
    private def populateGraphsAndLists() {
        this.reactionGraph = new ReactionInstanceGraph(this.main)
        this.connectivityGraph = new ConnectivityGraph(this.main, this.reactionGraph)
        this.reactions = this.reactionGraph.nodes

        // Ports are populated during the construction of the connectivity graph
        this.ports = this.connectivityGraph.ports
        // Populate state variables by traversing reactor instances
        populateStateVars(this.main)
    }
    
    /**
     * Generate the Uclid model and a runner script.
     */
    protected def generateModel() {     
        // Generate main.ucl and print to file
        code = new StringBuilder()
        var filename = outputDir.resolve("main.ucl").toString
        generateMain()
        JavaGeneratorUtils.writeSourceCodeToFile(getCode, filename)
        
        // Generate run.sh
        code = new StringBuilder()
        filename = outputDir.resolve("run.sh").toString
        generateRunScript()
        JavaGeneratorUtils.writeSourceCodeToFile(getCode, filename)
    }
    
    protected def generateMain() {
        pr('''
        /*******************************
         * Auto-generated UCLID5 model *
         ******************************/
        ''')
        pr('')
        
        // Start the main module
        pr('''
        module main {
        ''')
        pr('')
        indent()
        
        // Print timing semantics
        pr_timing_semantics()

        // Reaction IDs and state variables
        pr_rxn_ids_and_state_vars()
        
        // Trace definition
        pr_trace_def_and_helper_macros()
        
        // Topology
        pr_topological_abstraction()
        
        // Reactor semantics
        pr_reactor_semantics()
        
        // Reaction contracts
        pr_reaction_contracts()

        // Connections
        pr_connections()

        // Topology
        pr_program_topology()

        // Initial Condition
        pr_initial_condition()

        // Properties
        pr_properties()

        // K-induction
        pr_k_induction()

        // Control block
        pr_control_block()
        
        unindent()
        pr('}')
    }

    def pr_timing_semantics() {
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
        pr('')
    }

    // FIXME: generate custom code.
    def pr_rxn_ids_and_state_vars() {
        // [dynamic] Encode the components and
        // the logical delays present in a reactor system.
        pr('''
        /**********************************
         * Reaction IDs & State Variables *
         *********************************/
        
        //////////////////////////
        // Application Specific
        ''')
        
        /* Enumerate over all reactions */
        pr('''
        // Reaction ids
        type rxn_t = enum {
        
        ''')
        indent()
        var i = 0;
        for (rxn : reactions) {
            // Print a list of reaction IDs.
            // Add a comma if not last.
            pr(rxn.toId +
                (i++ == this.reactionGraph.nodes.size - 1 ? "" : ","))
        }
        unindent()
        pr('}')

        /* State variables and ports */
        // FIXME: expand to data types other than integer
        pr('''
        type state_t = {
        ''')
        indent()
        println(this.stateVars)
        println(this.ports)
        i = 0;
        for (v : this.stateVars) {
            pr(
                "integer"
                + ((ports.size() == 0 && i++ == stateVars.size - 1) ? "" : ",")
                + " \t// " + stateVarToString(v, ".")
            )
        }
        i = 0;
        for (p : this.ports) {
            pr(
                "integer"
                + ((i++ == ports.size - 1) ? "" : ",")
                + " \t// " + p.getFullName
            )
        }
        unindent()
        pr('''
        };
        //////////////////////////
        ''')
        pr('')
    }

    // FIXME: Accept k as an argument.
    def pr_trace_def_and_helper_macros() {
        pr('''
        /********************
        * Trace Definition *
        *******************/
        const START : integer = 0;
        const END : integer = «k»;
        
        define in_range(num : integer) : boolean
        = num >= START && num <= END;
        
        type step_t = integer;
        type event_t = { rxn_t, tag_t, state_t };
        ''')
        pr('''
        // Generate «k+1» events for «k»-induction.
        ''')
        pr("type trace_t = {")
        indent()
        for (var i = 0; i < k + 1; i++) {
            pr("event_t" +
                (i == k ? "" : ","))
        }
        unindent()
        pr('};')
        var varSize = this.stateVars.size + this.ports.size
        var integerInit = "0, ".repeat(varSize)
        integerInit = integerInit.substring(0, integerInit.length - 2)
        pr('''
        // mark the start of the trace.
        var start : timestamp_t;
        // assume(start == 0);
        
        // declare the trace
        var trace : trace_t;

        /*****************
         * helper macros *
         ****************/
        ''')

        pr('''
        // helper macro that returns an element based on index
        define get(tr : trace_t, i : step_t) : event_t =
        ''')
        for (var j = 0; j < varSize; j++) {
            pr('''
            if (i == «j») then tr._«j+1» else (
            ''')
        }
        pr('''
        { NULL, inf(), { «integerInit» } } «")".repeat(varSize)»;
        ''')

        pr('''
        define elem(i : step_t) : event_t
        = get(trace, i);
        
        // projection macros
        define rxn(i : step_t) : rxn_t     = elem(i)._1;
        define   g(i : step_t) : tag_t     = elem(i)._2;
        define   s(i : step_t) : state_t   = elem(i)._3;

        // application specific: state variables
        define isNULL(i : step_t)  : boolean = rxn(i) == NULL;
        ''')
        pr('')
        // FIXME: Finish generating 
        var i = 0;
        for (v : this.stateVars) {
            pr('''
            define «stateVarToString(v, "_")»(s : state_t) : integer = s._«i+1»;
            ''')
            i++;
        }
        for (p : this.ports) {
            pr('''
            define «p.getFullName.replace(".", "_")»(s : state_t) : integer = s._«i+1»;
            ''')
            i++;
        }
        pr('')
    }
    
    def pr_topological_abstraction() {
        pr('''
        /***************************
        * Topological Abstraction *
        ***************************/
        ''')

        // Generate the delay macro
        pr('''
        // Delay macro
        define delay(r1, r2 : rxn_t) : interval_t =
        ''')
        indent()
        var i = 0
        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, ConnectivityInfo> entry :
            connectivityGraph.connectivity.entrySet()) {
            pr('''
            if (r1 == «entry.getKey.getKey.toId» && r2 == «entry.getKey.getValue.toId») then nsec(«entry.getValue.delay») else (
            ''')
            i++;
        }
        pr('inf()')
        var closingBrackets = ')'.repeat(i)
        pr('''
        «closingBrackets»;
        ''')
        unindent()

        // Non-federated "happened-before"
        // FIXME: Would be nice if UCLID supports recursion of macros.
        // Happened-before relation defined for a local reactor.
        // Used to preserve trace ordering.
        pr('''
        // Non-federated "happened-before"
        define hb(e1, e2 : event_t) : boolean =
        = tag_earlier(e1._2, e2._2)
            || (tag_same(e1._2, e2._2) && (
        ''')
        indent()
        indent()
        i = 0
        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, ConnectivityInfo> entry :
            connectivityGraph.connectivity.entrySet()) {
            var upstream    = entry.getKey.getKey.toId
            var downstream  = entry.getKey.getValue.toId
            pr('''
            «i == 0 ? "" : "|| "»(e1._1 == «upstream» && e2._1 == «downstream»)
            ''')
            i++;
        }
        unindent()
        unindent()
        pr('));')    

        pr('''
        define startup_triggers(n : rxn_t) : boolean
        = // if startup is within frame, put the events in the trace.
            ((start == 0) ==> (exists (i : integer) :: in_range(i)
                && rxn(i) == n && tag_same(g(i), zero())))
            // Can ONLY be triggered at (0,0).
            && !(exists (j : integer) :: in_range(j) && rxn(j) == n
                && !tag_same(g(j), zero()));

        // Note: The current formulation of "triggers" precludes
        //       partial reaction triggering chain.
        // This includes the possibility that upstream does NOT output.
        define triggers_via_logical_action
            (upstream, downstream : rxn_t, delay : interval_t) : boolean
        = forall (i : integer) :: in_range(i)
            ==> (rxn(i) == downstream 
                ==> (exists (j : integer) :: in_range(j)
                    && rxn(j) == upstream 
                    && g(i) == tag_schedule(g(j), delay)));

        define triggers_via_logical_connection
            (upstream, downstream : rxn_t, delay : interval_t) : boolean
        = forall (i : integer) :: in_range(i)
            ==> (rxn(i) == downstream 
                ==> (exists (j : integer) :: in_range(j)
                    && rxn(j) == upstream 
                    && g(i) == tag_delay(g(j), delay)));
        ''')
        pr('')
    }
    
    // Encode reactor semantics
    def pr_reactor_semantics() {
        pr('''
        /*********************
         * Reactor Semantics *
         *********************/
        /** transition relation **/
        // transition relation constrains future states
        // based on previous states.

        // Events are ordered by "happened-before" relation.
        axiom(forall (i, j : integer) :: (in_range(i) && in_range(j))
            ==> (hb(elem(i), elem(j)) ==> i < j));
        
        // the same event can only trigger once in a logical instant
        axiom(forall (i, j : integer) :: (in_range(i) && in_range(j))
            ==> ((rxn(i) == rxn(j) && i != j)
                ==> !tag_same(g(i), g(j))));

        // Tags should be positive
        axiom(forall (i : integer) :: (i > START && i <= END)
            ==> pi1(g(i)) >= 0);

        // Microsteps should be positive
        axiom(forall (i : integer) :: (i > START && i <= END)
            ==> pi2(g(i)) >= 0);
        ''')
        pr('')
    }
    
    // Reaction contracts
    def pr_reaction_contracts() {
        pr('''
        /**********************
         * Reaction Contracts *
         **********************/
        ''')
        pr('')
        for (rxn : this.reactions) {
            pr('''
            /* Pre/post conditions for «rxn.toId» */
            axiom(forall (i : integer) :: (i > START && i <= END) ==>
                (rxn(i) == «rxn.toId» ==>
            ''')
            indent()
            // FIXME: Use LF macros to fill in this part.
            // Currently users have to fill in manually.
            pr('true // Change "true" to pre/post conditions that hold for the reaction body.')
            unindent()
            pr('''
            ));
            ''')
            pr('')
        }
    }

    // Connections
    def pr_connections() {
        pr('''
        /***************
         * Connections *
         ***************/
         
        ''')
        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, ConnectivityInfo> entry :
            connectivityGraph.connectivity.entrySet()) {
            // Check if two reactions are linked by a connection
            // if so, the output port and the input port should
            // hold the same value.
            // FIXME: finish generating connections
            if (entry.getValue.isConnection) {
                var upstream    = entry.getKey.getKey.toId
                var downstream  = entry.getKey.getValue.toId
                pr('''
                // source.out -> component._in 
                axiom(forall (i : integer) :: (i >= START && i <= END)
                    ==> (
                        (rxn(i) == source_1 ==> component__in(s(i)) == source_out(s(i)))
                        && (rxn(i) != source_1 ==> component__in(s(i)) == component__in(s(i - 1)))
                    ));
                ''')
            }
        }
    }

    // Topology
    def pr_program_topology() {
        pr('''
        /********************
         * Program Topology *
         ********************/
        // FIXME: add template
         
        ''')
    }

    // Initial Condition
    def pr_initial_condition() {
        pr('''
        /*********************
         * Initial Condition *
         *********************/
        // FIXME: add template
        define initial_condition() : boolean
        = start == 0
            && rxn(0) == NULL
            && g(0) == {0, 0}
            ;
         
        ''')
    }

    // Properties
    def pr_properties() {
        pr('''
        // [placeholder] Add user-defined properties here.
         
        //////////////////////////////////////////////////
        ''')
        pr('')
    }

    // K-induction
    def pr_k_induction() {
        pr('''
        /***************
        * K-induction *
        ***************/
        // initialization
        property initialization : initial_condition() ==>
            (forall (i : integer) ::
                (i >= START && i <= END)
                    ==> (
                        inv(i)
                        && auxiliary_invariant(i)
                    ));

        // Note: state 0 needs to be unconstrained.
        // consecution
        property consecution : (forall (i : integer) ::
            (i >= START && i < END) ==> (inv(i) && auxiliary_invariant(i)))
                ==> (inv(END) && auxiliary_invariant(END));
        ''')
        pr('')
    }

    // Control block
    def pr_control_block() {
        pr('''
        control {
            v = unroll(0);
            check;
            print_results;
            v.print_cex;
        }
        ''')
        pr('')
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

    protected def String stateVarToString(Pair<ReactorInstance, StateVar> s, String delimiter) {
        if (delimiter.equals("_"))
            return (s.getKey.getFullName + delimiter + s.getValue.name).replace(".", "_")
        else return (s.getKey.getFullName + delimiter + s.getValue.name)
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