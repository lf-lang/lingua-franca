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
import java.util.HashMap
import java.util.LinkedHashMap
import java.util.LinkedList
import java.util.List
import java.util.Map
import java.util.Set
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.generator.PortInstance
import org.lflang.generator.JavaGeneratorUtils
import org.lflang.lf.Action
import org.lflang.lf.After
import org.lflang.lf.At
import org.lflang.lf.Attribute
import org.lflang.lf.AttrParm
import org.lflang.lf.AtomicProp
import org.lflang.lf.Code
import org.lflang.lf.Conjunction
import org.lflang.lf.Difference
import org.lflang.lf.Disjunction
import org.lflang.lf.Equivalence
import org.lflang.lf.Expr
import org.lflang.lf.Finally
import org.lflang.lf.Globally
import org.lflang.lf.Implication
import org.lflang.lf.LogicalPrimary
import org.lflang.lf.LTLUnary
import org.lflang.lf.Negation
import org.lflang.lf.Next
import org.lflang.lf.PortPresent
import org.lflang.lf.PriorVarComp
import org.lflang.lf.Product
import org.lflang.lf.Quotient
import org.lflang.lf.ReactionComp
import org.lflang.lf.RelExpr
import org.lflang.lf.ScheduleAction
import org.lflang.lf.SetPort
import org.lflang.lf.Simultaneous
import org.lflang.lf.StateVar
import org.lflang.lf.Sum
import org.lflang.lf.Time
import org.lflang.lf.Until
import org.lflang.lf.VarComp
import org.lflang.lf.VarRef
import org.lflang.lf.WeakUntil
import org.lflang.lf.Within
import org.lflang.CausalityInfo
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.Target
import org.lflang.TimeUnit
import org.lflang.TimeValue

import static extension org.lflang.ASTUtils.*

/**
 * Generator for Uclid models.
 * 
 * @author Shaokai Lin {@literal <shaokai@eecs.berkeley.edu>}
 */
class UclidGenerator extends GeneratorBase {
    
    ////////////////////////////////////////////
    //// Public variables
    
    // The output directory where the model is stored
    Path outputDir
    
    // The reaction graph upon which the causality graph is built
    var ReactionInstanceGraph reactionGraph

    // Data structures for storing info about the runtime instances
    var Set<ReactionInstance>   reactions

    // String lists storing variable names of different types
    var List<String> variables          = new LinkedList
    var List<String> triggers           = new LinkedList
    var List<String> triggerPresence    = new LinkedList
    
    // This list of "unreferenced variables (resp. triggers)"
    // is used to track the variables (resp. triggers)
    // that are not referenced by a formula, and default
    // those variables (resp. triggers) to
    // their previous values (resp. false).
    var List<String> unrefVars
    var List<String> unrefTriggers

    // Downstream ports needs to be tracked and be taken out of
    // the unrefVars list, because of the connection axioms.
    var List<String> downstreamPorts = new LinkedList

    // The causality graph captures counterfactual causality
    // relations between adjacent reactions.
    var CausalityMap causalityMap

    // K-induction steps
    int k

    // Initial quantified variable index
    int initQFIdx = 0

    // Initial prefix index
    String initPrefixIdx = "i"

    // Data structures for storing properties
    var List<String>                        properties  = new ArrayList
    var HashMap<String, List<Attribute>>    propertyMap = new LinkedHashMap
    var HashMap<String, List<Attribute>>    auxInvMap   = new LinkedHashMap
    var HashMap<String, Attribute>          boundMap    = new LinkedHashMap
    
    // Constructor
    new(FileConfig fileConfig, ErrorReporter errorReporter) {
        super(fileConfig, errorReporter)
    }
    
    ////////////////////////////////////////////
    //// Private variables
    
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa,
        IGeneratorContext context) {
        
        // The following generates code needed by all the reactors.
        super.doGenerate(resource, fsa, context)

        // Check for the specified engine.
        if (this.targetConfig.verification !== null) {
            if (this.targetConfig.verification.engine !== null) {
                // Check for the specified k-induction steps, otherwise defaults to 1.
                this.k = this.targetConfig.verification.induction
                switch (this.targetConfig.verification.engine) {
                    case "uclid": {
                        generateModel(resource, fsa, context)
                    }
                    default: {
                        throw new RuntimeException("The specified engine is not supported.")
                    }
                }
            }
        } else {
            // If verification flag is not specified, exit the generator.
            return
        }
    }

    def void generateModel(Resource resource, IFileSystemAccess2 fsa,
        IGeneratorContext context) {

        // FIXME: Does this handle the federated reactor in case of federated execution?    
        for (federate : federates) {
            // Build the instantiation tree if a main reactor is present.
            if (this.mainDef !== null) {
                if (this.main === null) {
                    // Recursively build instances. This is done once because
                    // it is the same for all federates.
                    this.main = new ReactorInstance(
                        mainDef.reactorClass.toDefinition,
                        this.errorReporter
                    )
                }   
            } else {
                println("WARNING: No main reactor detected. No model is generated.")
                return
            }
        }  

        // Build reaction instance graph and causality graph
        populateGraphsAndLists()
        
        // Create the "src-gen" directory if it doesn't yet exist.
        var dir = fileConfig.getSrcGenPath.toFile
        if (!dir.exists()) dir.mkdirs()
        
        // Create the "model" directory if it doesn't yet exist.
        dir = fileConfig.getSrcGenPath.resolve("model").toFile
        if (!dir.exists()) dir.mkdirs()
        outputDir = Paths.get(dir.getAbsolutePath)
        println("The models will be located in: " + outputDir)

        // Identify properties and generate a model for each property.
        var mainAttr = this.main.reactorDefinition.getAttributes
        if (mainAttr.length != 0) {
            for (attr : mainAttr) {
                // Extract the property name.
                // Add to the list if it doesn't exist.
                var property = attr.getAttrParms.get(0).getStr.replaceAll("^\"|\"$", "")
                if (!this.properties.contains(property)) {
                    this.properties.add(property)
                }
                // Check the type of the attribute,
                // then populate the hashmap.
                switch attr.getAttrName.toString {
                    case 'property' : {
                        if (this.propertyMap.get(property) === null) {
                            this.propertyMap.put(property, new LinkedList)
                        }
                        this.propertyMap.get(property).add(attr)
                    }
                    case 'aux' : {
                        if (this.auxInvMap.get(property) === null) {
                            this.auxInvMap.put(property, new LinkedList)
                        }
                        this.auxInvMap.get(property).add(attr)
                    }
                    case 'bound' : {
                        if (this.boundMap.get(property) === null) {
                            this.boundMap.put(property, attr)
                        } else {
                            println("WARNING: Redundant bound specification for the same property.")
                        }
                    }
                }
            }
        } else {
            println("WARNING: No main reactor attribute detected. No model is generated.")
            return
        }

        // Generate a Uclid model for each property.
        for (property : this.properties) {
            printModelToFile(property)
        }

        // Generate runner script
        code = new StringBuilder()
        var filename = outputDir.resolve("run.sh").toString
        generateRunnerScript()
        JavaGeneratorUtils.writeSourceCodeToFile(getCode, filename)
    }

    /**
     * @brief Recursively add state variables to the stateVars list.
     * @param reactor A reactor instance from which the search begins.
     */
    private def void populateStateVars(ReactorInstance reactor, List<Pair<ReactorInstance, StateVar>> list) {      
        var stateVars = reactor.reactorDefinition.getStateVars
        // Prefix state vars with reactor name
        // and add them to the list
        for (s : stateVars) {
            list.add(new Pair(reactor, s))
        }
        // Populate state variables recursively
        for (child : reactor.children) {
            populateStateVars(child, list)
        }
    }

    /**
     * Populate the data structures.
     */
    private def populateGraphsAndLists() {
        // Construct graphs
        this.reactionGraph = new ReactionInstanceGraph(this.main)
        this.causalityMap = new CausalityMap(this.main, this.reactionGraph)

        // Collect reactions from the reaction graph.
        this.reactions = this.reactionGraph.nodes

        // Collect ports and state variables from the program.
        var Set<PortInstance> portInstances
        var Set<ActionInstance> actionInstances
        var List<Pair<ReactorInstance, StateVar>> stateVarInstances
            = new ArrayList<Pair<ReactorInstance, StateVar>>
        
        // Ports and actions are populated during
        // the construction of the causality graph
        portInstances = this.causalityMap.ports
        actionInstances = this.causalityMap.actions

        // Populate state variables by traversing reactor instances
        populateStateVars(this.main, stateVarInstances)

        // Generate a list of variables, which will be used in code generation.
        // The list constitutes all the variables in the state_t type.
        for (p : portInstances) {
            this.variables.add(p.getFullNameWithJoiner("_"))
            this.triggers.add(p.getFullNameWithJoiner("_"))
            this.triggerPresence.add(p.getFullNameWithJoiner("_") + "_is_present")
        }
        for (a : actionInstances) {
            this.variables.add(a.getFullNameWithJoiner("_"))
            this.triggers.add(a.getFullNameWithJoiner("_"))
            this.triggerPresence.add(a.getFullNameWithJoiner("_") + "_is_present")
        }
        for (v : stateVarInstances) {
            this.variables.add(stateVarFullNameWithJoiner(v, "_"))
        }
    }
    
    /**
     * Generate the Uclid model and a runner script.
     */
    protected def printModelToFile(String property) {     
        // Generate main.ucl and print to file
        code = new StringBuilder()
        var filename = outputDir.resolve("property_" + property + ".ucl").toString
        generateMain(property)
        JavaGeneratorUtils.writeSourceCodeToFile(getCode, filename)
    }
    
    protected def generateMain(String property) {
        // FIXME: Auto-calculate the bound
        // Extract the property bound out of the attribute.
        var bound      = this.boundMap.get(property)
        var int boundValue
        if (bound !== null) {
            boundValue = Integer.parseInt(bound.getAttrParms.get(1).getInt)
        } else {
            throw new RuntimeException("Property bound is not provided for " + property)
        }
        var traceLength  = boundValue + this.k
        println("Trace length: " + traceLength)
        println("Bound value: " + boundValue)
        println("k: " + this.k)

        pr('''
        /*******************************
         * Auto-generated UCLID5 model *
         ******************************/
        ''')
        // Start the main module
        pr('''
        module main {
        ''')
        newline()
        indent()
        
        // Print timing semantics
        pr_timing_semantics()

        // Reaction IDs and state variables
        pr_rxn_ids_and_state_vars()
        
        // Trace definition
        pr_trace_def_and_helper_macros(traceLength)
        
        // Topology
        pr_topological_abstraction()
        
        // Reactor semantics
        pr_reactor_semantics()

        // Connections
        pr_connections_and_actions()

        // Topology
        pr_program_topology()

        // Initial Condition
        pr_initial_condition()

        // Reactor contracts
        pr_reactor_contracts()

        // Reaction contracts
        pr_reaction_contracts()

        // K-induction
        var conjunctList = this.propertyMap.get(property)
        var auxInvList   = this.auxInvMap.get(property)
        pr_k_induction(property, conjunctList, auxInvList, boundValue)

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
        type timestamp_t = integer;                     // The unit is nanoseconds
        type microstep_t = integer;
        type tag_t = {
            timestamp_t,
            microstep_t
        };
        type interval_t  = tag_t;
        
        // Projection macros
        define pi1(t : tag_t) : timestamp_t = t._1;     // Get timestamp from tag
        define pi2(t : tag_t) : microstep_t = t._2;     // Get microstep from tag
        
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
        
        // mstep() produces a mstep delay. zero() produces no delay.
        define tag_schedule(t : tag_t, i : interval_t) : tag_t
        = if (!isInf(t) && !isInf(i) && pi1(i) == 0 && pi2(i) == 1)
            then { pi1(t), pi2(t) + 1 } // microstep delay
            else ( if (!isInf(t) && !isInf(i) && pi1(i) == 0 && pi2(i) == 0)
                    then t // no delay
                    else (
                        if (!isInf(t) && pi1(i) > 0 && !isInf(i))
                        then { pi1(t) + pi1(i), 0 }
                        else inf()
                    ));
        
        // Deprecated.
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
        newline()
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
        for (rxn : this.reactions) {
            // Print a list of reaction IDs.
            // Add a comma if not last.
            pr(rxn.getFullNameWithJoiner('_') + ',')
        }
        pr('NULL')
        unindent()
        pr('};')

        /* State variables and ports */
        // FIXME: expand to data types other than integer
        pr('''
        type state_t = {
        ''')
        indent()
        if (this.variables.size > 0) {
            var i = 0;
            for (v : this.variables) {
                pr('''integer«((i++ == this.variables.size - 1) ? "" : ",")»    // «v»''')
            }
        } else {
            pr('''
            // There are no ports or state variables.
            // Insert a dummy integer to make the model compile.
            integer
            ''')
        }
        unindent()
        pr('};')
        pr('''
        // State variable projection macros
        ''')
        var i = 1;
        for (v : this.variables) {
            pr('''define «v»(s : state_t) : integer = s._«i++»;''')
        }
        newline()

        // A boolean tuple that stores whether a port is present.
        // FIXME: Extend to actions.
        pr('''
        type trigger_t = {
        ''')
        indent()
        if (this.triggers.size > 0) {
            i = 0;
            for (t : this.triggers) {
                pr('''boolean«((i++ == this.triggers.size - 1) ? "" : ",")»    // «t»''')
            }
        } else {
            pr('''
            // There are no ports or state variables.
            // Insert a dummy integer to make the model compile.
            boolean
            ''')
        }
        unindent()
        pr('};')
        pr('''
        // Trigger presence projection macros
        ''')
        i = 1;
        for (t : this.triggerPresence) {
            pr('''define «t»(t : trigger_t) : boolean = t._«i++»;''')
        }
        newline()
    }

    def pr_trace_def_and_helper_macros(int traceLength) {
        pr('''
        /********************
         * Trace Definition *
         *******************/
        const START : integer = 0;
        const END : integer = «traceLength-1»;
        
        define in_range(num : integer) : boolean
        = num >= START && num <= END;
        
        type step_t = integer;
        type event_t = { rxn_t, tag_t, state_t, trigger_t };
        ''')
        pr('')
        pr('''
        // Create a bounded trace of «traceLength» events.
        ''')
        pr("type trace_t = {")
        indent()
        for (var i = 0; i < traceLength; i++) {
            pr("event_t" +
                (i == traceLength - 1 ? "" : ","))
        }
        unindent()
        pr('};')
        newline()

        pr('''
        // mark the start of the trace.
        var start : timestamp_t;
        
        // declare the trace
        var trace : trace_t;

        /*****************
         * Helper Macros *
         ****************/
        ''')

        // Generate a getter for the finite trace.
        var String stateInit
        if (this.variables.size > 0) {
            stateInit = "0, ".repeat(this.variables.size)
            stateInit = stateInit.substring(0, stateInit.length - 2)
        } else {
            // Initialize a dummy variable just to make the code compile.
            stateInit = "0"
        }

        var String triggerInit
        if (this.triggers.size > 0) {
            triggerInit = "false, ".repeat(this.triggers.size)
            triggerInit = triggerInit.substring(0, triggerInit.length - 2)
        } else {
            // Initialize a dummy variable just to make the code compile.
            triggerInit = "false"
        }

        pr('''
        // helper macro that returns an element based on index
        define get(tr : trace_t, i : step_t) : event_t =
        ''')
        for (var j = 0; j < traceLength; j++) {
            pr('''
            if (i == «j») then tr._«j+1» else (
            ''')
        } 
        pr('''
        { NULL, inf(), { «stateInit» }, { «triggerInit» } } «")".repeat(traceLength)»;
        ''')
        newline()
        pr('''
        define elem(i : step_t) : event_t
        = get(trace, i);
        
        // projection macros
        define rxn      (i : step_t) : rxn_t        = elem(i)._1;
        define g        (i : step_t) : tag_t        = elem(i)._2;
        define s        (i : step_t) : state_t      = elem(i)._3;
        define t        (i : step_t) : trigger_t    = elem(i)._4;
        define isNULL   (i : step_t) : boolean      = rxn(i) == NULL;
        ''')
        newline()
    }
    
    def pr_topological_abstraction() {
        pr('''
        /***************************
         * Topological Abstraction *
         ***************************/
        ''')
        // Non-federated "happened-before"
        // FIXME: Need to compute the transitive closure.
        // Happened-before relation defined for a local reactor.
        // Used to preserve trace ordering.
        pr('''
        // Non-federated "happened-before"
        define hb(e1, e2 : event_t) : boolean
        = tag_earlier(e1._2, e2._2)
        ''')
        indent()
        indent()
        var i = 0
        var str = '''
        || (tag_same(e1._2, e2._2) && (
        '''
        // Exclude timer and startup by checking if the downstream reaction is null.
        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, List<CausalityInfo>> entry :
            causalityMap.causality.entrySet.filter[it.getKey.getKey !== it.getKey.getValue]) {
            var upstream    = entry.getKey.getKey.getFullNameWithJoiner('_')
            var downstream  = entry.getKey.getValue.getFullNameWithJoiner('_')
            str += '''
            «i == 0 ? "" : "|| "»(e1._1 == «upstream» && e2._1 == «downstream»)
            '''
            i++;
        }
        // If there are no counterfactual reaction pairs,
        // simply put a "true" there.
        if (i != 0) {
            pr(str)
            pr('));')
        } else {
            pr(';')
        }
        unindent()
        unindent()
        newline() 

        pr('''
        define startup_triggers(n : rxn_t) : boolean
        =   // if startup is within frame, put the events in the trace.
            ((start == 0) ==> (exists (i : integer) :: in_range(i)
                && rxn(i) == n && tag_same(g(i), zero())));

        //// Encoding the behavior of timers
        define is_multiple_of(a, b : integer) : boolean
        = exists (c : integer) :: b * c == a;
        
        define is_closest_starting_point(t : tag_t, period, offset : integer) : boolean
        = (exists (c : integer) :: (period * c) + offset == pi1(t)
            // Tick at the next valid instant.
            && (period * (c - 1) + offset) < start)     
            // Timer always has mstep of 0.
            && pi2(t) == 0;

        // Can directly use index as HB since this only applies to events
        // on the same federate.
        define is_latest_invocation_in_same_fed_wrt(a, b : integer) : boolean
        = !(exists (c : integer) :: in_range(c)
            && rxn(c) == rxn(a) && a < c && c < b);
        
        define timer_triggers(_rxn : rxn_t, offset, period : integer) : boolean =
            // 1. If the initial event is within frame, show it.
            (exists (i : integer) :: in_range(i)
            && rxn(i) == _rxn
            && is_closest_starting_point(g(i), period, offset))
            // 2. The SPACING between two consecutive timer firings is the period.
            // FIXME: Is the use of two in_range() here appropriate?
            // Shaokai: Seems so to me, since the first state is not actually constrained.
            && (forall (i, j : integer) :: (in_range(i) && in_range(j) && i < j
                && rxn(i) == _rxn && rxn(j) == _rxn
                // ...and there does not exist a 3rd invocation in between
                && !(exists (k : integer) :: rxn(k) == _rxn && i < k && k < j))
                    ==> g(j) == tag_schedule(g(i), {period, 0}))
            // 3. There does not exist other events in the same federate that 
            // differ by the last timer invocation by g(last_timer) + period.
            // In other words, this axiom ensures a timer fires when it needs to.
            //
            // a := index of the offending event that occupy the spot of a timer tick.
            // b := index of non-timer event on the same federate
            // both in_range's are needed due to !(exists), which turns into a forall.
            && !(exists (b, a : integer) :: in_range(a) && in_range(b)
                && rxn(b) != _rxn
                // && _id_same_fed(elem(b), {_id, zero()})
                && rxn(a) == _rxn
                && (is_latest_invocation_in_same_fed_wrt(a, b)
                    && tag_later(g(b), tag_schedule(g(a), {period, 0}))));
        ''')
        newline()
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

        // Begin the frame at the start time specified.
        define start_frame(i : step_t) : boolean =
            (tag_same(g(i), {start, 0}) || tag_later(g(i), {start, 0}));
        axiom(forall (i : integer) :: (i > START && i <= END)
            ==> start_frame(i));

        // NULL events should appear in the suffix, except for START.
        axiom(forall (j : integer) :: (j > START && j <= END) ==> (
            (rxn(j)) != NULL) ==> 
                (forall (i : integer) :: (i > START && i < j) ==> (rxn(i) != NULL)));

        // When a NULL event occurs, the state stays the same.
        axiom(forall (j : integer) :: (j > START && j <= END) ==> (
            (rxn(j) == NULL) ==> (s(j) == s(j-1))
        ));
        ''')
        newline()
    }

    /** 
     * Axioms for connections
     * 
     * Connections should not involve reaction invocations.
     * There needs to be axioms on ports triggering reactions.
     * 
     * Note that the value of upstream ports is constrained
     * by reaction contracts, hence it is not specified here.
     */
    def pr_connections_and_actions() {
        pr('''
        /***************************
         * Connections and Actions *
         ***************************/
        ''')
        newline()
        // Handle cases that have an upstream and a downstream reaction.
        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, List<CausalityInfo>> entry : causalityMap.causality.entrySet) {
            for (causality : entry.getValue) {
                var upstreamRxn = entry.getKey.getKey.getFullNameWithJoiner('_')
                var upstreamPort = causality.upstreamPort?.getFullNameWithJoiner('_')
                var upstreamPortIsPresent = upstreamPort + '_is_present'
                var downstreamRxn = entry.getKey.getValue.getFullNameWithJoiner('_')
                var downstreamPort = causality.downstreamPort?.getFullNameWithJoiner('_')
                var downstreamPortIsPresent = downstreamPort + '_is_present'
                var trigger = causality.triggerInstance?.getFullNameWithJoiner('_')
                var triggerIsPresent = trigger + '_is_present'
                var isPhysical = causality.isPhysical
                var delay = causality.delay
                // Collecting downstream ports here so that they can be removed
                // from unreferenced variables.
                // FIXME: Do this when generating the causality graph.
                //        Doing this here seems awkward. 
                this.downstreamPorts.add(downstreamPort)
                switch causality.type {
                    case "connection" : {
                        // Upstream port connections to downstream port.
                        pr('''
                        // «upstreamPort» «isPhysical? "~>" : "->"» «downstreamPort»
                        axiom(forall (i : integer) :: (i > START && i <= END) ==> (
                        ''')
                        indent()
                        pr('''
                        // If «upstreamPort» is present then «downstreamPort» will be present
                        // with the same value after some fixed delay of «delay» nanoseconds.
                        («upstreamPortIsPresent»(t(i)) ==> (
                            exists (j : integer) :: j > i && j <= END
                            && «downstreamPortIsPresent»(t(j))
                            && «downstreamPort»(s(j)) == «upstreamPort»(s(i))
                        ''')
                        if (!isPhysical) {
                            indent()
                            pr('''&& g(j) == tag_schedule(g(i), «delay == 0 ? "zero()" : '''nsec(«delay»)'''»)''')
                            unindent()
                        }
                        pr('))')
                        pr('''
                        // If «downstreamPort» is present, there exists an «upstreamPort».
                        // This additional term establishes a one-to-one relationship in timing.
                        && («downstreamPortIsPresent»(t(i)) ==> (
                            exists (j : integer) :: j >= START && j < i
                            && «upstreamPortIsPresent»(t(j))
                        ''')
                        if (!isPhysical) {
                            indent()
                            pr('''&& g(i) == tag_schedule(g(j), «delay == 0 ? "zero()" : '''nsec(«delay»)'''»)''')
                            unindent()
                        }
                        pr('))')
                        pr('''
                        // If «downstreamPort» is not present, then value stays the same.
                        && (!«downstreamPortIsPresent»(t(i)) ==> (
                            «downstreamPort»(s(i)) == «downstreamPort»(s(i-1))
                        ))
                        ''')
                        unindent()
                        pr('));')
                        newline()
                    }
                    // For the barrier case (external reaction triggers contained reaction),
                    // the predicate set() is needed.
                    // FIXME: Currently the barrier model is blocked by the presence variable.
                    // case "barrier",
                    //
                    // The scheduling of action is handled by reaction contracts.
                    // The user calls the predicate schedule(action, value, additional_delay)
                    case "action",
                    case "timer" : {
                        pr('''
                        // If «trigger» is present, then there exists a «upstreamRxn»
                        // that scheduled it.
                        axiom(forall (i : integer) :: (i > START && i <= END) ==> ( true
                        ''')
                        indent()
                        if (!isPhysical) {
                            pr('''
                            // If «trigger» is present, there exists an «upstreamRxn».
                            // This additional term establishes a one-to-one relationship in timing.
                            && («triggerIsPresent»(t(i)) ==> (
                                exists (j : integer) :: j >= START && j < i
                                && rxn(j) == «upstreamRxn»
                            ''')
                            indent()
                            // Subtle point:
                            // For "barrier", there is no delay.
                            // For "action" and "barrier", there is at least a microstep delay.
                            pr('''&& g(i) == tag_schedule(g(j), «delay == 0 ? (causality.type == "barrier" ? "zero()" : "mstep()") : '''nsec(«delay»)'''»)''')
                            unindent()
                        }
                        pr('))')
                        pr('''
                        // If «trigger» is not present, then value stays the same.
                        && (!«triggerIsPresent»(t(i)) ==> (
                            «trigger»(s(i)) == «trigger»(s(i-1))
                        ))
                        ''')
                        unindent()
                        pr('));')
                        newline()
                    }
                }
            }
        }

        /**
         * FIXME: The following implementation is very awkward.
         * Must be fixed before being merged in production.
         * 
         * 1. For each reaction, gather the triggers.
         * 2. Check if any of the triggers is used by another reaction.
         */
        pr('''
        /********************************
         * Reactions and Their Triggers *
         ********************************/
        ''')
        newline()
        for (rxn : this.reactions) {
            pr('''
            // «rxn.getFullName» is invoked when any of it triggers are present.
            axiom(forall (i : integer) :: (i > START && i <= END) ==> ((
                false
            ''')
            indent()
            // If the reaction is being triggered
            for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, List<CausalityInfo>> entry : causalityMap.causality.entrySet.filter[it.getKey.getValue.getFullName == rxn.getFullName]) {
                for (causality : entry.getValue) {
                    var Object trigger
                    // Connections
                    if (causality.downstreamPort !== null) {
                        trigger = causality.downstreamPort
                    } else if (causality.triggerInstance !== null) {
                        trigger = causality.triggerInstance
                    } else {
                        throw new RuntimeException('Unreachable')
                    }

                    // Check if this port/startup/action/timer is used by another reaction.
                    var rxnWithSameTrigger = new LinkedList<ReactionInstance>
                    // Iterate through the rest of the reactions in the program.
                    for (_rxn : this.reactions.filter[it != rxn]) {
                        // Iterate through entries that have _rxn as the triggered reaction.
                        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, List<CausalityInfo>> _entry : causalityMap.causality.entrySet.filter[it.getKey.getValue.getFullName == _rxn.getFullName]) {
                            for (_causality : _entry.getValue) {
                                // If reaction _rxn has the same trigger as reaction rxn.
                                if ((_causality.downstreamPort !== null && _causality.downstreamPort == trigger) || (_causality.triggerInstance !== null && _causality.triggerInstance == trigger)) {
                                    rxnWithSameTrigger.add(_rxn)
                                }
                            }
                        }
                    }

                    var String isPresentStr
                    if (trigger instanceof PortInstance) {
                        isPresentStr = '''«trigger.getFullNameWithJoiner('_')»_is_present(t(i))'''
                    } else if (trigger instanceof TriggerInstance) {
                        if (trigger.isStartup) {
                            isPresentStr = '''g(i) == zero()'''
                        } else {
                            isPresentStr = '''«trigger.getFullNameWithJoiner('_')»_is_present(t(i))'''
                        }
                    } 
                    if (rxnWithSameTrigger.length > 0) {
                        for (__rxn : rxnWithSameTrigger) {
                            pr('''
                            || («isPresentStr» && rxn(i) != «__rxn.getFullNameWithJoiner("_")»)
                            ''')
                        }
                    } else {
                        pr('''|| «isPresentStr»''')
                    }
                }
            }
            unindent()
            pr('''
            ) <==> (rxn(i) == «rxn.getFullNameWithJoiner("_")»)));
            ''')
            newline()
        }
    }

    // Topology
    def pr_program_topology() {
        pr('''
        /**********************
         * Startup & Shutdown *
         **********************/         
        ''')
        newline()
        for (Map.Entry<Pair<ReactionInstance, ReactionInstance>, List<CausalityInfo>> entry :
            causalityMap.causality.entrySet) {
            for (causality : entry.getValue.filter[it.type == "startup"]) {
                var upstreamRxn     = entry.getKey.getKey
                var upstreamName    = upstreamRxn.getFullName
                var upstreamId      = upstreamRxn.getFullNameWithJoiner('_')
                if (causality.type.equals("startup")) {
                    pr('''
                    // «upstreamName» is triggered by startup.
                    axiom(startup_triggers(«upstreamId»));
                    ''')
                }
                newline()
            }
        }
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
        ''')
        indent()
        for (v : this.variables) {
            pr('''&& «v»(s(0)) == 0''')
        }
        pr(';')
        newline()
        unindent()
    }

    def pr_reactor_contracts() {
        pr('''
        /*********************
         * Reactor Contracts *
         *********************/
        ''')
        newline()
        // FIXME: Iterate over reactor instances not definitions.
        // for (r : this.reactors.filter[!it.main]) {
        //     var reactorAttr = r.getAttributes
        //     if (reactorAttr.length != 0) {
        //         for (attr : reactorAttr) {
        //             if (attr.getAttrName.toString.equals("inv")) {
        //                 // Extract the invariant out of the attribute.
        //                 var inv = LTL2FOL(attr.getAttrParms.get(0), initQFIdx, initPrefixIdx)
        //                 // Print line number
        //                 prSourceLineNumber(attr)
        //                 pr('''
        //                 /* Input/output relations for «r.getName» */
        //                 axiom(forall (i : integer) :: (i > START && i <= END) ==>
        //                 ''')
        //                 indent()
        //                 pr(inv)
        //                 unindent()
        //                 pr('''
        //                 ));
        //                 ''')
        //                 newline()
        //             }
        //         }
        //     }
        // }
    }

    // Reaction contracts
    def pr_reaction_contracts() {
        pr('''
        /**********************
         * Reaction Contracts *
         **********************/
        ''')
        newline()
        // Iterate over reaction INSTANCES.
        for (rxn : this.reactions) {
            // Initialize a new list of unreferenced variables.
            this.unrefVars      = new LinkedList<String>(this.variables.clone)
            this.unrefTriggers  = new LinkedList<String>(this.triggerPresence.clone)
            // Important:
            // i >= START since the contract needs to be enforced at state 0.
            // But does this constrain the 1st state?
            // It does not if we add the reaction contracts to auxiliary invariants.
            // i >= START is an implicit way of doing so.
            pr('''
            /* Pre/post conditions for «rxn.getFullName» */
            axiom(forall (i : integer) :: (i >= START && i <= END) ==>
                (rxn(i) == «rxn.getFullNameWithJoiner('_')» ==> ( true
            ''')
            indent()
            var attrList = rxn.definition.getAttributes
            if (attrList.length != 0) {
                for (attr : attrList) {
                    if (attr.getAttrName.toString.equals("inv")) {
                        // Extract the invariant out of the attribute.
                        // LTL2FOL recursively removes referenced variables from unrefVars
                        // and referenced ports from unrefTriggers.
                        var inv = LTL2FOL(attr.getAttrParms.get(0), initQFIdx, initPrefixIdx, "0", rxn.parent)
                        // Print line number
                        prSourceLineNumber(attr)
                        pr('''&& («inv»)''')
                    }
                }
            }

            /* Generate default behaviors for variables and ports */

            // Further remove downstream ports and triggers from the unref list,
            // since there are connection axioms and reaction contracts
            // that dictate their behavior.
            this.unrefVars.removeAll(this.downstreamPorts)
            this.unrefVars.removeAll(this.triggers)

            // The rest of the variables stay the same.
            for (v : this.unrefVars) {
                pr('''&& «v»(s(i)) == «v»(s(i-1))''')
            }

            // Remove the input ports that can trigger this reaction
            // from the list of unrefTriggers.
            for (t : rxn.triggers) {
                if (t instanceof PortInstance || t instanceof ActionInstance) {
                    this.unrefTriggers.remove(t.getFullNameWithJoiner("_") + "_is_present")
                }
            }
            // The remaining triggers stay absent.
            for (t : this.unrefTriggers) {
                pr('''&& !«t»(t(i))''')
            }
            unindent()
            pr('''
            )));
            ''')
            newline()
        }
    }

    // Properties
    def pr_k_induction(String propertyName, List<Attribute> conjunctList, List<Attribute> auxInvList, int boundValue) {
        pr('''
        /************
         * Property *
         ************/
        ''')
        pr('''
        // trace length = k + N
        const k : integer = «this.k»;       // Induction steps
        const N : integer = «boundValue»;   // The property bound
        ''')
        newline()

        // Print the property in the form of a conjunction.
        pr('''
        // The FOL property translated from user-defined LTL property
        define P(i : step_t) : boolean =
            true
        ''')
        indent()
        for (conjunct : conjunctList) {
            // Extract the invariant out of the attribute.
            var formula = LTL2FOL(conjunct.getAttrParms.get(1), initQFIdx, initPrefixIdx, "0", this.main)
            // Print line number
            prSourceLineNumber(conjunct)
            pr('''
            && («formula»)
            ''')
        }
        unindent()
        pr(";")
        newline()

        // Print auxiliary invariants.
        pr('''
        // Auxiliary invariant
        define aux_inv(i : integer) : boolean =
            // Add this here, so that in the consecution step,
            // the first state respects start.
            start_frame(i)
        ''');
        indent()
        if (auxInvList !== null) {
            for (auxInv : auxInvList) {
                // Extract the invariant out of the attribute.
                // var formula = auxInv.getAttrParms.get(1).replaceAll("^\"|\"$", "")
                var formula = LTL2FOL(auxInv.getAttrParms.get(1), initQFIdx, initPrefixIdx, "0", this.main)
                // Print line number
                prSourceLineNumber(auxInv)
                pr('''
                && («formula»)
                ''')
            }
        }
        unindent()
        pr(";")
        newline()

        // Compose the property and the auxiliary invariants.
        pr('''
        // Strengthened property
        define Q(i : step_t) : boolean =
            P(i) && aux_inv(i);

        // Helper macro for temporal induction
        define Globally_Q(start, end : step_t) : boolean =
            (forall (i : integer) :: (i >= start && i <= end) ==> Q(i));
        ''')
        newline()

        // Print k-induction formulae.
        pr('''
        /***************
         * K-Induction *
         ***************/
        // Initiation
        property initiation_«propertyName» : initial_condition() ==>
            Globally_Q(0, k);

        // Consecution
        property consecution_«propertyName» : 
            Globally_Q(0, k) ==> Q(k+1);
        ''')
        newline()
    }

    // Control block
    def pr_control_block() {
        pr('''
        control {
            v = bmc(0);
            check;
            print_results;
            v.print_cex;
        }
        ''')
    }
    
    protected def generateRunnerScript() {
        pr('''
        #!/bin/bash
        set -e # Terminate on error
        
        echo '*** Setting up smt directory'
        rm -rf ./smt/ && mkdir -p smt
        
        echo '*** Generating SMT files from UCLID5'
        uclid -g "smt/output" $1
        
        echo '*** Append (get-model) to each file'
        ls smt | xargs -I {} bash -c 'echo "(get-model)" >> smt/{}'
        
        echo '*** Running Z3'
        ls smt | xargs -I {} bash -c 'echo "Checking {}" && z3 -T:120 ./smt/{}'
        ''')
    }
    
    /////////////////////////////////////////////////
    //// Helper functions

    protected def String stateVarFullNameWithJoiner(Pair<ReactorInstance, StateVar> s, String joiner) {
        if (joiner.equals("_"))
            return (s.getKey.getFullName + joiner + s.getValue.name).replace(".", "_")
        else return (s.getKey.getFullName + joiner + s.getValue.name)
    }

    protected def newline() {
        pr('')
    }

    protected def long Time2Nsec(Time time) {
        var interval = time.getInterval
        var TimeUnit unit = TimeUnit.fromName(time.getUnit)
        var TimeValue timeValue = new TimeValue(interval, unit)
        return timeValue.toNanoSeconds
    }

    /**
     * Leave a marker in the generated code that indicates the original line
     * number in the LF source.
     * @param eObject The node.
     */
    override prSourceLineNumber(EObject eObject) {
        if (eObject instanceof Code) {
            pr(code, '''//// Line «NodeModelUtils.getNode(eObject).startLine +1» in the LF program''')
        } else {
            pr(code, '''//// Line «NodeModelUtils.getNode(eObject).startLine» in the LF program''')
        }
    }

    /**
     * Recursively generate string based on the type of the AST node.
     *
     * @param ASTNode The AST node.
     * @param QFIdx A monotonically increasing index number for quantified variables.
     *              Available to be used. E.g, i0, i1, i2, ...
     */
    protected def dispatch String LTL2FOL(AttrParm ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Base case: If formula is provided in String, then return verbatim.
        if (ASTNode.getStr !== null) {
            return ASTNode.getStr
        }

        // Otherwise, recurse on the LTL formula
        return LTL2FOL(ASTNode.getLtl, QFIdx, prefixIdx, prevIdx, instance)
    }

    protected def dispatch String LTL2FOL(Equivalence ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If right is null, continue recursion
        if (ASTNode.getRight === null) {
            return LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)
        }
        // Otherwise, create the <==> symbol
        return '''(«LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)») <==> («LTL2FOL(ASTNode.getRight, QFIdx, prefixIdx, prevIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(Implication ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If right is null, continue recursion
        if (ASTNode.getRight === null) {
            return '''(«LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)»)'''
        }
        // Otherwise, create the ==> symbol
        return '''(«LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)») ==> («LTL2FOL(ASTNode.getRight, QFIdx, prefixIdx, prevIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(Disjunction ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var str = ""
        var i = 0; // Used to check if end of the list has been reached
        for (t : ASTNode.getTerms) {
            str += '''(«LTL2FOL(t, QFIdx, prefixIdx, prevIdx, instance)»)«((i++ == ASTNode.getTerms.size - 1) ? "" : "||")»'''
        }
        return str
    }

    protected def dispatch String LTL2FOL(Conjunction ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var str = ""
        var i = 0; // Used to check if end of the list has been reached
        for (t : ASTNode.getTerms) {
            str += '''(«LTL2FOL(t, QFIdx, prefixIdx, prevIdx, instance)»)«((i++ == ASTNode.getTerms.size - 1) ? "" : "&&")»'''
        }
        return str
    }

    protected def dispatch String LTL2FOL(Until ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If right is null, continue recursion
        if (ASTNode.getRight === null) {
            return LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)
        }
        // Otherwise, create the Until formula
        return '''exists (j«QFIdx» : integer) :: j«QFIdx» >= «prefixIdx» && j«QFIdx» <= («prefixIdx» + N) && («LTL2FOL(ASTNode.getRight, QFIdx+1, ('j'+QFIdx), prefixIdx, instance)») && (forall (i«QFIdx» : integer) :: (i«QFIdx» >= «prefixIdx» && i«QFIdx» < j«QFIdx») ==> («LTL2FOL(ASTNode.getLeft, QFIdx+1, ('i'+QFIdx), ('j'+QFIdx), instance)»))'''
    }

    protected def dispatch String LTL2FOL(WeakUntil ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If right is null, continue recursion
        if (ASTNode.getRight === null) {
            return LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)
        }
        // Otherwise, create the WeakUntil formula
        return '''(«LTL2FOL((ASTNode as Until), QFIdx, prefixIdx, prevIdx, instance)») || («LTL2FOL((ASTNode.getLeft as Globally), QFIdx, prefixIdx, prevIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(LTLUnary ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Pass onto the next stage.
        return '''(«LTL2FOL(ASTNode.getFormula, QFIdx, prefixIdx, prevIdx, instance)»)'''
    }
    
    protected def dispatch String LTL2FOL(Negation ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Create the Negation formula.
        return '''!(«LTL2FOL(ASTNode.getFormula, QFIdx, prefixIdx, prevIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(Globally ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Create the Globally formula.
        return '''forall (i«QFIdx» : integer) :: (i«QFIdx» >= «prefixIdx» && i«QFIdx» <= («prefixIdx» + N)) ==> («LTL2FOL(ASTNode.getFormula, QFIdx+1, ('i'+QFIdx), prefixIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(Finally ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Create the Finally formula.
        return '''exists (i«QFIdx» : integer) :: i«QFIdx» >= «prefixIdx» && i«QFIdx» <= («prefixIdx» + N) && («LTL2FOL(ASTNode.getFormula, QFIdx+1, ('i'+QFIdx), prefixIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(Next ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Create the Next formula.
        return '''(«LTL2FOL(ASTNode.getFormula, QFIdx, ('(' + prefixIdx + '+1)'), prefixIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(LogicalPrimary ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Pass onto AtomicProp
        return '''(«LTL2FOL(ASTNode.getAtom, QFIdx, prefixIdx, prevIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(AtomicProp ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // Check primitive
        if (ASTNode.getPrimitive !== null) {
            switch ASTNode.getPrimitive {
                case 'true',
                case 'True' : {
                    return 'true'
                }
                case 'false',
                case 'False' : {
                    return 'false'
                }
            }
        }
        if (ASTNode.getComponent !== null) {
            return LTL2FOL(ASTNode.getComponent, QFIdx, prefixIdx, prevIdx, instance)
        }
        if (ASTNode.getTiming !== null) {
            return LTL2FOL(ASTNode.getTiming, QFIdx, prefixIdx, prevIdx, instance)
        }
        throw new RuntimeException("Unreachable")
    }

    /*
     * If the variable has a container specified, then use that.
     * If the container is a reactor, try to get the reactor instance
     * by iterating through the main reactor instance and match by name.
     * Once that is obtained, check if ID is a port, state var, or reaction.
     *
     * E.g.:
     *      (Port)      r.in, r.out
     *      (State var) r.s, r.count
     *      (Reaction)  r.reaction[0], r.reaction[1]
     * 
     * Time:
     *      r.reaction[0] ==> (r.reaction[0] /\ at(1 sec))
     *      r.reaction[0] ==> (r.reaction[0] /\ before(1 sec))
     *      r.reaction[0] ==> (r.reaction[0] /\ after(1 sec))
     *
     * Ports and reaction names can be obtained
     * using <port/rxnInstance>.getFullNameWithJoiner('_')
     */
    protected def dispatch String LTL2FOL(VarComp ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If there is no container, build the state var function call.
        var reactor = instance as ReactorInstance
        if (ASTNode.variable !== null) {
            var varName = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»'''
            this.unrefVars.remove(varName)
            return '''«varName»(s(«prefixIdx»))'''
        }
        // Otherwise，traverse the ReactorInstance tree.
        for (container : ASTNode.containers) {
            for (child : reactor.children) {
                if (child.getName == container.name) {
                    reactor = child
                }
            }
        }
        var varName = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getId»'''
        this.unrefVars.remove(varName)
        return '''«varName»(s(«prefixIdx»))'''
    }

    protected def dispatch String LTL2FOL(PriorVarComp ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If there is no container, build the state var function call.
        var reactor = instance as ReactorInstance
        if (ASTNode.variable !== null) {
            return '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»(s(«prefixIdx»-1))'''
        }
        // Otherwise，traverse the ReactorInstance tree.
        for (container : ASTNode.containers) {
            for (child : reactor.children) {
                if (child.getName == container.name) {
                    reactor = child
                }
            }
        }
        // Note the -1.
        return '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getId»(s(«prefixIdx»-1))'''
    }

    protected def dispatch String LTL2FOL(ReactionComp ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var rxnIndex = Integer.parseInt(ASTNode.rxnId) - 1 // Minus 1 to convert rxnId to index
        var reactor = instance as ReactorInstance
        for (container : ASTNode.containers) {
            for (child : reactor.children) {
                if (child.getName == container.name) {
                    reactor = child
                }
            }
        }
        return '''rxn(«prefixIdx») == «reactor.reactions.get(rxnIndex).getFullNameWithJoiner('_')»'''
    }

    protected def dispatch String LTL2FOL(SetPort ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If there is no container, build the state var function call.
        var reactor = instance as ReactorInstance
        var value = Integer.parseInt(ASTNode.getValue)
        var String varName
        var String varPresence

        if (ASTNode.variable !== null) {
            varName = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»'''
            varPresence = '''«varName»_is_present'''
            this.unrefVars.remove(varName)
            this.unrefTriggers.remove(varPresence)
        } else {
            // Otherwise，traverse the ReactorInstance tree.
            for (container : ASTNode.containers) {
                for (child : reactor.children) {
                    if (child.getName == container.name) {
                        reactor = child
                    }
                }
            }
            varName = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getId»'''
            varPresence = '''«varName»_is_present'''
            this.unrefVars.remove(varName)
            this.unrefTriggers.remove(varPresence)
        }
        
        return '''«varPresence»(t(«prefixIdx»)) && «varName»(s(«prefixIdx»)) == «value»'''
    }

    protected def dispatch String LTL2FOL(PortPresent ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If there is no container, build the state var function call.
        var reactor = instance as ReactorInstance
        var varName = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»'''
        var varPresence = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»_is_present'''
        // Not removing the variable while stating its presence might be too restrictive.
        // On this other hand, this might create spurious CEX, which requires more aux. inv.
        this.unrefVars.remove(varName)
        this.unrefTriggers.remove(varPresence)
        return '''«varPresence»(t(«prefixIdx»))'''
    }

    protected def dispatch String LTL2FOL(ScheduleAction ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        // If there is no container, build the state var function call.
        var reactor = instance as ReactorInstance
        var value = Integer.parseInt(ASTNode.getValue)
        
        // FIXME: We currently only support fixed delays.
        // To support additional delay, we need to add more
        // variables to the states. Otherwise, it is hard to
        // write the axioms on one-to-one correspondence.
        // var additionalDelay = Time2Nsec(ASTNode.getDelay)

        var varName = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»'''
        var varPresence = '''«reactor.getFullNameWithJoiner('_')»_«ASTNode.getVariable.getName»_is_present'''
        this.unrefVars.remove(varName)
        this.unrefTriggers.remove(varPresence)

        var ActionInstance actionInstance
        for (a : reactor.actions) {
            if (a.getName == ASTNode.getVariable.getName) {
                actionInstance = a
            }
        }
        if (actionInstance === null) {
            throw new RuntimeException("Could not find an ActionInstance.")
        }
        var isPhysical = actionInstance.isPhysical
        var minDelay = actionInstance.minDelay.toNanoSeconds
        // var totalDelay = minDelay + additionalDelay
        var recurrent = false
        if (this.causalityMap.recurrentActions.contains(actionInstance))
            recurrent = true

        // Using («prefixIdx» < END) to prevent blocking.
        if (isPhysical) {
            return '''(«prefixIdx» < END) ==> (exists (i«QFIdx» : integer) :: i«QFIdx» > «prefixIdx» && «varPresence»(t(i«QFIdx»)) && «varName»(s(i«QFIdx»)) == «value»)'''
        } else {
            if (recurrent) {
                return '''((«prefixIdx» < END) ==> (exists (i«QFIdx» : integer) :: i«QFIdx» > «prefixIdx» && «varPresence»(t(i«QFIdx»)) && «varName»(s(i«QFIdx»)) == «value» && g(«'i'+QFIdx») == tag_schedule(g(«prefixIdx»), «minDelay == 0? "mstep()" : '''nsec(«minDelay»)'''»)))
                || (
                    !(exists (x«QFIdx» : integer) :: x«QFIdx» > «prefixIdx» && «varPresence»(t(x«QFIdx»)))
                    && !(exists (y«QFIdx» : integer) :: y«QFIdx» > «prefixIdx» && y«QFIdx» <= END && !«varPresence»(t(y«QFIdx»)) && tag_later(g(y«QFIdx»), tag_schedule(g(«prefixIdx»), nsec(«minDelay»))))
                )'''
            } else {
                return '''((«prefixIdx» < END) ==> (exists (i«QFIdx» : integer) :: i«QFIdx» > «prefixIdx» && «varPresence»(t(i«QFIdx»)) && «varName»(s(i«QFIdx»)) == «value» && g(«'i'+QFIdx») == tag_schedule(g(«prefixIdx»), «minDelay == 0? "mstep()" : '''nsec(«minDelay»)'''»)))'''
            }
        }
    }

    protected def dispatch String LTL2FOL(Simultaneous ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        return '''tag_same(g(«prefixIdx»), g(«prevIdx»))'''
    }

    protected def dispatch String LTL2FOL(At ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var interval = ASTNode.getTime.getInterval
        var TimeUnit unit = TimeUnit.fromName(ASTNode.getTime.getUnit)
        var TimeValue timeValue = new TimeValue(interval, unit)
        var nanoSec = timeValue.toNanoSeconds
        return '''tag_same(g(«prefixIdx»), tag_schedule(g(«prevIdx»), nsec(«nanoSec»)))'''
    }

    protected def dispatch String LTL2FOL(Within ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var interval = ASTNode.getTime.getInterval
        var TimeUnit unit = TimeUnit.fromName(ASTNode.getTime.getUnit)
        var TimeValue timeValue = new TimeValue(interval, unit)
        var nanoSec = timeValue.toNanoSeconds
        return '''tag_earlier(g(«prefixIdx»), tag_schedule(g(«prevIdx»), nsec(«nanoSec»)))'''
    }

    protected def dispatch String LTL2FOL(After ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var interval = ASTNode.getTime.getInterval
        var TimeUnit unit = TimeUnit.fromName(ASTNode.getTime.getUnit)
        var TimeValue timeValue = new TimeValue(interval, unit)
        var nanoSec = timeValue.toNanoSeconds
        return '''tag_later(g(«prefixIdx»), tag_schedule(g(«prevIdx»), nsec(«nanoSec»)))'''
    }

    protected def dispatch String LTL2FOL(RelExpr ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        return '''(«LTL2FOL(ASTNode.getLeft, QFIdx, prefixIdx, prevIdx, instance)»)«ASTNode.getRelOp» («LTL2FOL(ASTNode.getRight, QFIdx, prefixIdx, prevIdx, instance)»)'''
    }

    protected def dispatch String LTL2FOL(Expr ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        if (ASTNode.getSum !== null) {
            return '''(«LTL2FOL(ASTNode.getSum, QFIdx, prefixIdx, prevIdx, instance)»)'''
        }
        if (ASTNode.getComponent !== null) {
            return '''(«LTL2FOL(ASTNode.getComponent, QFIdx, prefixIdx, prevIdx, instance)»)'''
        }
        return ASTNode.getInt.toString // Int cannot be null. Return it last.
    }
    
    protected def dispatch String LTL2FOL(Sum ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var str = ""
        var i = 0; // Used to check if end of the list has been reached
        for (t : ASTNode.getTerms) {
            str += '''(«LTL2FOL(t, QFIdx, prefixIdx, prevIdx, instance)»)«((i++ == ASTNode.getTerms.size - 1) ? "" : "+")»'''
        }
        return str
    }

    protected def dispatch String LTL2FOL(Difference ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var str = ""
        var i = 0; // Used to check if end of the list has been reached
        for (t : ASTNode.getTerms) {
            str += '''(«LTL2FOL(t, QFIdx, prefixIdx, prevIdx, instance)»)«((i++ == ASTNode.getTerms.size - 1) ? "" : "-")»'''
        }
        return str
    }

    protected def dispatch String LTL2FOL(Product ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var str = ""
        var i = 0; // Used to check if end of the list has been reached
        for (t : ASTNode.getTerms) {
            str += '''(«LTL2FOL(t, QFIdx, prefixIdx, prevIdx, instance)»)«((i++ == ASTNode.getTerms.size - 1) ? "" : "*")»'''
        }
        return str
    }

    protected def dispatch String LTL2FOL(Quotient ASTNode, int QFIdx, String prefixIdx, String prevIdx, Object instance) {
        var str = ""
        var i = 0; // Used to check if end of the list has been reached
        for (t : ASTNode.getTerms) {
            str += '''(«LTL2FOL(t, QFIdx, prefixIdx, prevIdx, instance)»)«((i++ == ASTNode.getTerms.size - 1) ? "" : "/")»'''
        }
        return str
    }

    /////////////////////////////////////////////////
    //// Functions from generatorBase
    
    override getTarget() {
        return Target.C // FIXME: How to make this target independent? Target.ALL does not work.
    }

    override supportsGenerics() {
        return false
    }

    // Intentionally preserve delays.
    override transformDelays() {
        return
    }

    override generateDelayBody(Action action, VarRef port) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override generateForwardBody(Action action, VarRef port) {
        throw new UnsupportedOperationException("TODO: auto-generated method stub")
    }
    
    override generateDelayGeneric() {
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