/* A helper class for analyzing the AST. */

/*************
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy

import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.List
import java.util.Set
import org.eclipse.emf.common.util.EList
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.TerminalRule
import org.eclipse.xtext.nodemodel.ILeafNode
import org.eclipse.xtext.nodemodel.impl.CompositeNode
import org.eclipse.xtext.nodemodel.impl.HiddenLeafNode
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.eclipse.xtext.resource.XtextResource
import org.icyphy.generator.FederateInstance
import org.icyphy.generator.GeneratorBase
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.ArraySpec
import org.icyphy.linguaFranca.Code
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.ImportedReactor
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.ReactorDecl
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.TypeParm
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.WidthSpec

/**
 * A helper class for modifying and analyzing the AST.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 */
class ASTUtils {
    
    /**
     * The Lingua Franca factory for creating new AST nodes.
     */
    public static val factory = LinguaFrancaFactory.eINSTANCE
    
    /**
     * Find connections in the given resource that have a delay associated with them, 
     * and reroute them via a generated delay reactor.
     * @param resource The AST.
     * @param generator A code generator.
     */
    static def insertGeneratedDelays(Resource resource, GeneratorBase generator) {
        // The resulting changes to the AST are performed _after_ iterating 
        // in order to avoid concurrent modification problems.
        val oldConnections = new LinkedList<Connection>()
        val newConnections = new LinkedHashMap<Reactor, List<Connection>>()
        val delayInstances = new LinkedHashMap<Reactor, List<Instantiation>>()
        val delayClasses = new LinkedHashSet<Reactor>()
        
        // Iterate over the connections in the tree.
        for (container : resource.allContents.toIterable.filter(Reactor)) {
            for (connection : container.connections) {
                if (connection.delay !== null) {
                    val parent = connection.eContainer as Reactor
                    // Assume all the types are the same, so just use the first on the right.
                    val type = (connection.rightPorts.get(0).variable as Port).type
                    val delayClass = getDelayClass(type, delayClasses, container, resource, generator)
                    val generic = generator.supportsGenerics
                            ? generator.getTargetType(InferredType.fromAST(type))
                            : ""
                    // If the left or right has a multiport or bank, then create a bank
                    // of delays with an inferred width.
                    // FIXME: If the connection already uses an inferred width on
                    // the left or right, then this will fail because you cannot
                    // have an inferred width on both sides.
                    val isWide = connection.isWide
                    val delayInstance = getDelayInstance(delayClass, connection.delay, generic, isWide)

                    // Stage the new connections for insertion into the tree.
                    var connections = newConnections.get(parent)
                    if(connections === null) connections = new LinkedList()
                    connections.addAll(connection.rerouteViaDelay(delayInstance))
                    newConnections.put(parent, connections)

                    // Stage the original connection for deletion from the tree.
                    oldConnections.add(connection)

                    // Stage the newly created delay reactor instance for insertion
                    var instances = delayInstances.get(parent)
                    if(instances === null) instances = new LinkedList()
                    instances.addAll(delayInstance)
                    delayInstances.put(parent, instances)
                }
            }
        }
        // Remove old connections; insert new ones.
        oldConnections.forEach [ connection |
            (connection.eContainer as Reactor).connections.remove(connection)
        ]
        newConnections.forEach [ reactor, connections |
            reactor.connections.addAll(connections)
        ]
        // Finally, insert the instances and, before doing so, assign them a unique name.
        delayInstances.forEach [ reactor, instantiations |
            instantiations.forEach [ instantiation |
                instantiation.name = reactor.getUniqueIdentifier("delay");
                reactor.instantiations.add(instantiation)
            ]
        ]
    }
    
    /**
     * Return true if any port on the left or right of the connection invoves
     * a bank of reactors or a multiport.
     * @param connection The connection.
     */
    private static def boolean isWide(Connection connection) {
        for (leftPort : connection.leftPorts) {
            if ((leftPort.variable as Port).widthSpec !== null
                || leftPort.container?.widthSpec !== null
            ) {
                return true
            }
        }
        for (rightPort : connection.rightPorts) {
            if ((rightPort.variable as Port).widthSpec !== null
                || rightPort.container?.widthSpec !== null
            ) {
                return true
            }
        }
        return false
    }
    
    /**
     * Take a connection and reroute it via an instance of a generated delay
     * reactor. This method returns a list to new connections to substitute
     * the original one.
     * @param connection The connection to reroute.
     * @param delayInstance The delay instance to route the connection through.
     */
    private static def List<Connection> rerouteViaDelay(Connection connection, 
            Instantiation delayInstance) {
        
        val connections = new LinkedList<Connection>()
               
        val upstream = factory.createConnection
        val downstream = factory.createConnection
        val input = factory.createVarRef
        val output = factory.createVarRef
        
        val delayClass = delayInstance.reactorClass.toDefinition
        
        // Establish references to the involved ports.
        input.container = delayInstance
        input.variable = delayClass.inputs.get(0)
        output.container = delayInstance
        output.variable = delayClass.outputs.get(0)
        upstream.leftPorts.addAll(connection.leftPorts)
        upstream.rightPorts.add(input)
        downstream.leftPorts.add(output)
        downstream.rightPorts.addAll(connection.rightPorts)

        connections.add(upstream)
        connections.add(downstream)
        return connections
    }
    
    /**
     * Create a new instance delay instances using the given reactor class.
     * The supplied time value is used to override the default interval (which
     * is zero).
     * If the target supports parametric polymorphism, then a single class may
     * be used for each instantiation, in which case a non-empty string must
     * be supplied to parameterize the instance.
     * A default name ("delay") is assigned to the instantiation, but this
     * name must be overridden at the call site, where checks can be done to
     * avoid name collisions in the container in which the instantiation is
     * to be placed. Such checks (or modifications of the AST) are not
     * performed in this method in order to avoid causing concurrent
     * modification exceptions. 
     * @param delayClass The class to create an instantiation for
     * @param value A time interval corresponding to the desired delay
     * @param generic A string that denotes the appropriate type parameter, 
     *  which should be null or empty if the target does not support generics.
     * @param isWide True to create a variable-width width specification.
     */
    private static def Instantiation getDelayInstance(Reactor delayClass, 
            Value time, String generic, boolean isWide) {
        val delayInstance = factory.createInstantiation
        delayInstance.reactorClass = delayClass
        if (!generic.isNullOrEmpty) {
            val typeParm = factory.createTypeParm
            typeParm.literal = generic
            delayInstance.typeParms.add(typeParm)
        }
        if (isWide) {
            val widthSpec = factory.createWidthSpec
            delayInstance.widthSpec = widthSpec
            widthSpec.ofVariableLength = true
        }
        val delay = factory.createAssignment
        delay.lhs = delayClass.parameters.get(0)
        delay.rhs.add(time.copy)
        delayInstance.parameters.add(delay)
        delayInstance.name = "delay" // This has to be overridden.
        
        return delayInstance
                
    }
    
    /**
     * Return a synthesized AST node that represents the definition of a delay
     * reactor. Depending on whether the target supports generics, either this
     * method will synthesize a generic definition and keep returning it upon
     * subsequent calls, or otherwise, it will synthesize a new definition for 
     * each new type it hasn't yet created a compatible delay reactor for. All
     * the classes generated so far are passed as an argument to this method,
     * and newly created definitions are accumulated as a side effect of
     * invoking this method. For each invocation, if no existing definition
     * exists that can handle the given type, a new definition is created, 
     * it is added to the set generated classes, and it is returned.
     * @param type The type the delay class must be compatible with.
     * @param generatedClasses Set of class definitions already generated.
     * @param container The first container that needs this class.
     * @param resource The eCore resource.
     * @param generator A code generator.
     */
    private static def Reactor getDelayClass(
        Type type, 
        Set<Reactor> generatedClasses,
        Reactor container,
        Resource resource,
        GeneratorBase generator
    ) {
        
        val className = generator.supportsGenerics ? 
            GeneratorBase.GEN_DELAY_CLASS_NAME : {
                val id = Integer.toHexString(
                    InferredType.fromAST(type).toText.hashCode)
                '''«GeneratorBase.GEN_DELAY_CLASS_NAME»_«id»'''
            }
            
        // Only add class definition if it is not already there.
        val classDef = generatedClasses.findFirst[it|it.name.equals(className)]
        if (classDef !== null) {
            return classDef
        }
        
        val delayClass = factory.createReactor
        val delayParameter = factory.createParameter
        val action = factory.createAction
        val triggerRef = factory.createVarRef
        val effectRef = factory.createVarRef
        val input = factory.createInput
        val output = factory.createOutput
        val inRef = factory.createVarRef
        val outRef = factory.createVarRef
            
        val r1 = factory.createReaction
        val r2 = factory.createReaction
        
        delayParameter.name = "delay"
        delayParameter.type = factory.createType
        delayParameter.type.id = generator.targetTimeType
        val defaultValue = factory.createValue
        defaultValue.literal = generator.timeInTargetLanguage(
            new TimeValue(0, TimeUnit.NONE))
        delayParameter.init.add(defaultValue)
        
        // Name the newly created action; set its delay and type.
        action.name = "act"
        action.minDelay = factory.createValue
        action.minDelay.parameter = delayParameter
        action.origin = ActionOrigin.LOGICAL
                
        if (generator.supportsGenerics) {
            action.type = factory.createType
            action.type.id = "T"
        } else {
            action.type = type.copy
        }
        
        input.name = "inp"
        input.type = action.type.copy
        
        output.name = "out"
        output.type = action.type.copy
        
        // Establish references to the involved ports.
        inRef.variable = input
        outRef.variable = output
        
        // Establish references to the action.
        triggerRef.variable = action
        effectRef.variable = action
        
        // Add the action to the reactor.
        delayClass.name = className
        delayClass.actions.add(action)

        // Configure the second reaction, which reads the input.
        r1.triggers.add(inRef)
        r1.effects.add(effectRef)
        r1.code = factory.createCode()
        r1.code.body = generator.generateDelayBody(action, inRef)
    
        // Configure the first reaction, which produces the output.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        r2.code = factory.createCode()
        r2.code.body = generator.generateForwardBody(action, outRef)    
    
        // Add the reactions to the newly created reactor class.
        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
            
        delayClass.reactions.add(r2)
        delayClass.reactions.add(r1)

        // Add a type parameter if the target supports it.
        if (generator.supportsGenerics) {
            val parm = factory.createTypeParm
            parm.literal = generator.generateDelayGeneric()
            delayClass.typeParms.add(parm)
        }
        
        delayClass.inputs.add(input)
        delayClass.outputs.add(output)
        delayClass.parameters.add(delayParameter)
        
        generatedClasses.add(delayClass)
        
        // Finally, add the class definition just prior to the
        // container reactor in the resource.
        // Contained reactors are normally declared before container
        // reactors. But we don't want them to be too much before
        // because conceivably they could depend on preamble code
        // that defines data types in previously defined reactors.
        val model = findModel(resource)
        val position = reactorPosition(container, model)
        if (position < 0) {
            throw new RuntimeException("INTERNAL ERROR: Cannot find " + container + " in the model.")
        }
        model.reactors.add(position, delayClass)
        
        return delayClass
    }
    
    /**
     * Return the position of the specified reactor in the model contents.
     * Reactor class definitions are always in the top-level Model.
     * @param reactor The reactor class definition to find.
     * @param model The top-level model.
     * @return Return the position or -1 if it is not found.
     */
    static private def reactorPosition(Reactor reactor, Model model) {
        var position = 0
        for (candidate : model.reactors) {
            if (reactor === candidate) {
                return position
            }
            position++
        }
        return -1
    }
    
    /**
     * Return the top-level Model.
     */
    static def findModel(Resource resource) {
        var model = null as Model
        for (t : resource.contents) {
            if (t instanceof Model) {
                if (model !== null) {
                    throw new RuntimeException("There is more than one Model!")
                }
                model = t
            }
        }
        if (model === null) {
            throw new RuntimeException("No Model found!")
        }
        model
    }
    
    /** 
     * Replace the specified connection with a communication between federates.
     * @param connection The connection.
     * @param leftFederate The source federate.
     *  @param rightFederate The destination federate.
     */
    static def void makeCommunication(
        Connection connection, 
        FederateInstance leftFederate,
        FederateInstance rightFederate,
        GeneratorBase generator
    ) {
        val factory = LinguaFrancaFactory.eINSTANCE
        // Assume all the types are the same, so just use the first on the right.
        var type = (connection.rightPorts.get(0).variable as Port).type.copy
        val action = factory.createAction
        val triggerRef = factory.createVarRef
        val effectRef = factory.createVarRef
        val inRef = factory.createVarRef
        val outRef = factory.createVarRef
        val parent = (connection.eContainer as Reactor)
        val r1 = factory.createReaction
        val r2 = factory.createReaction
        
        // These reactions do not require any dependency relationship
        // to other reactions in the container.
        generator.makeUnordered(r1)
        generator.makeUnordered(r2)

        // Name the newly created action; set its delay and type.
        action.name = getUniqueIdentifier(parent, "networkMessage")
        // Handle connection delay.
        if (connection.delay !== null) {
            action.minDelay = factory.createValue
            action.minDelay.time = factory.createTime
            if (connection.delay.time !== null) {
                action.minDelay.time.interval = connection.delay.time.interval
                action.minDelay.time.unit = connection.delay.time.unit
            } else {
                action.minDelay.literal = connection.delay.literal
            }
        }
        action.type = type
        
        // The connection is 'physical' if it uses the ~> notation.
        if (connection.physical) {
            action.origin = ActionOrigin.PHYSICAL
        } else {
            action.origin = ActionOrigin.LOGICAL
        }
        
        // Record this action in the right federate.
        // The ID of the receiving port (rightPort) is the position
        // of the action in this list.
        val receivingPortID = rightFederate.networkMessageActions.length
        rightFederate.networkMessageActions.add(action)

        // Establish references to the action.
        triggerRef.variable = action
        effectRef.variable = action

        // Establish references to the involved ports.
        // FIXME: This does not support parallel connections yet!!
        if (connection.leftPorts.length > 1 || connection.rightPorts.length > 1) {
            throw new UnsupportedOperationException("FIXME: Parallel connections are not yet supported between federates.")
        }
        inRef.container = connection.leftPorts.get(0).container
        inRef.variable = connection.leftPorts.get(0).variable
        outRef.container = connection.rightPorts.get(0).container
        outRef.variable = connection.rightPorts.get(0).variable

        // Add the action to the reactor.
        parent.actions.add(action)

        // Configure the sending reaction.
        r1.triggers.add(inRef)
        r1.effects.add(effectRef)
        r1.code = factory.createCode()
        r1.code.body = generator.generateNetworkSenderBody(
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            action.inferredType
        )

        // Configure the receiving reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        r2.code = factory.createCode()
        r2.code.body = generator.generateNetworkReceiverBody(
            action,
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            action.inferredType
        )

        // Add the reactions to the parent.
        parent.reactions.add(r1)
        parent.reactions.add(r2)
    }
    
    /**
     * Produce a unique identifier within a reactor based on a
     * given based name. If the name does not exists, it is returned;
     * if does exist, an index is appended that makes the name unique.
     * @param reactor The reactor to find a unique identifier within.
     * @param name The name to base the returned identifier on.
     */
    static def getUniqueIdentifier(Reactor reactor, String name) {
        val vars = new LinkedHashSet<String>();
        reactor.allActions.forEach[it | vars.add(it.name)]
        reactor.allTimers.forEach[it | vars.add(it.name)]
        reactor.allParameters.forEach[it | vars.add(it.name)]
        reactor.allInputs.forEach[it | vars.add(it.name)]
        reactor.allOutputs.forEach[it | vars.add(it.name)]
        reactor.allStateVars.forEach[it | vars.add(it.name)]
        reactor.allInstantiations.forEach[it | vars.add(it.name)]
        
        var index = 0;
        var suffix = ""
        var exists = true
        while(exists) {
            val id = name + suffix
            if (vars.exists[it | it.equals(id)]) {
                suffix = "_" + index
                index++
            } else {
                exists = false;
            }
        }
        return name + suffix
    }
    
    /**
     * Given a "value" AST node, return a deep copy of that node.
     * @param original The original to create a deep copy of.
     * @return A deep copy of the given AST node.
     */
    private static def getCopy(Value original) {
        if (original !== null) {
            val clone = factory.createValue
            if (original.parameter !== null) {
                clone.parameter = original.parameter
            }
            if (original.time !== null) {
                clone.time = factory.createTime
                clone.time.interval = original.time.interval
                clone.time.unit = original.time.unit
            }
            if (original.literal !== null) {
                clone.literal = original.literal
            }
            if (original.code !== null) {
                clone.code = factory.createCode
                clone.code.body = original.code.body
            }
            return clone
        }
    }
    
    /**
     * Given a "type" AST node, return a deep copy of that node.
     * @param original The original to create a deep copy of.
     * @return A deep copy of the given AST node.
     */
    private static def getCopy(TypeParm original) {
        val clone = factory.createTypeParm
        if (!original.literal.isNullOrEmpty) {
            clone.literal = original.literal
        } else if (original.code !== null) {
                clone.code = factory.createCode
                clone.code.body = original.code.body
        }
        return clone
    }
    
    /**
     * Given a "type" AST node, return a deep copy of that node.
     * @param original The original to create a deep copy of.
     * @return A deep copy of the given AST node.
     */
     private static def getCopy(Type original) {
        if (original !== null) {
            val clone = factory.createType
            
            clone.id = original.id
            // Set the type based on the argument type.
            if (original.code !== null) {
                clone.code = factory.createCode
                clone.code.body = original.code.body
            } 
            if (original.stars !== null) {
                original.stars?.forEach[clone.stars.add(it)]
            }
                
            if (original.arraySpec !== null) {
                clone.arraySpec = factory.createArraySpec
                clone.arraySpec.ofVariableLength = original.arraySpec.
                    ofVariableLength
                clone.arraySpec.length = original.arraySpec.length
            }
            
            original.typeParms?.forEach[parm | clone.typeParms.add(parm.copy)]
            
            return clone
        }
    }
        
    ////////////////////////////////
    //// Utility functions for supporting inheritance
    
    /**
     * Given a reactor class, return a list of all its actions,
     * which includes actions of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Action> allActions(Reactor definition) {
        val result = new LinkedList<Action>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allActions)
        }
        result.addAll(definition.actions)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its connections,
     * which includes connections of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Connection> allConnections(Reactor definition) {
        val result = new LinkedList<Connection>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allConnections)
        }
        result.addAll(definition.connections)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its inputs,
     * which includes inputs of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Input> allInputs(Reactor definition) {
        val result = new LinkedList<Input>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allInputs)
        }
        result.addAll(definition.inputs)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its instantiations,
     * which includes instantiations of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Instantiation> allInstantiations(Reactor definition) {
        val result = new LinkedList<Instantiation>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allInstantiations)
        }
        result.addAll(definition.instantiations)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its outputs,
     * which includes outputs of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Output> allOutputs(Reactor definition) {
        val result = new LinkedList<Output>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allOutputs)
        }
        result.addAll(definition.outputs)
        return result
    }

    /**
     * Given a reactor class, return a list of all its parameters,
     * which includes parameters of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Parameter> allParameters(Reactor definition) {
        val result = new LinkedList<Parameter>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allParameters)
        }
        result.addAll(definition.parameters)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its reactions,
     * which includes reactions of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Reaction> allReactions(Reactor definition) {
        val result = new LinkedList<Reaction>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allReactions)
        }
        result.addAll(definition.reactions)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its state variables,
     * which includes state variables of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<StateVar> allStateVars(Reactor definition) {
        val result = new LinkedList<StateVar>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allStateVars)
        }
        result.addAll(definition.stateVars)
        return result
    }
    
    /**
     * Given a reactor class, return a list of all its timers,
     * which includes timers of base classes that it extends.
     * @param definition Reactor class definition.
     */
    def static List<Timer> allTimers(Reactor definition) {
        val result = new LinkedList<Timer>()
        for (base : definition.superClasses?:emptyList) {
            result.addAll(base.toDefinition.allTimers)
        }
        result.addAll(definition.timers)
        return result
    }

    ////////////////////////////////
    //// Utility functions for translating AST nodes into text

    /**
     * Translate the given code into its textual representation.
     * @param code AST node to render as string.
     * @return Textual representation of the given argument.
     */
    def static String toText(Code code) {
        if (code !== null) {
            val node = NodeModelUtils.getNode(code)
            if (node !== null) {
                val builder = new StringBuilder(
                    Math.max(node.getTotalLength(), 1))
                for (ILeafNode leaf : node.getLeafNodes()) {
                    builder.append(leaf.getText());
                }
                var str = builder.toString.trim
                // Remove the code delimiters (and any surrounding comments).
                // This assumes any comment before {= does not include {=.
                val start = str.indexOf("{=")
                val end = str.indexOf("=}", start)
                str = str.substring(start + 2, end)
                if (str.split('\n').length > 1) {
                    // multi line code
                    return str.trimCodeBlock
                } else {
                    // single line code
                    return str.trim
                }    
            } else if (code.body !== null) {
                // Code must have been added as a simple string.
                return code.body.toString
            }
        }
        return ""
    }
    
    def static toText(TypeParm t) {
        if (!t.literal.isNullOrEmpty) {
            t.literal
        } else {
            t.code.toText
        }
    }
    
    /**
     * Intelligently trim the white space in a code block.
	 * 
	 * The leading whitespaces of the first non-empty
	 * code line is considered as a common prefix across all code lines. If the
	 * remaining code lines indeed start with this prefix, it removes the prefix
	 * from the code line.
	 * 
     * For examples, this code
     * <pre>{@code 
     *        int test = 4;
     *        if (test == 42) {
     *            printf("Hello\n");
     *        }
     * }</pre>
     * will be trimmed to this:
     * <pre>{@code 
     * int test = 4;
     * if (test == 42) {
     *     printf("Hello\n");
     * }
     * }</pre>
     * 
     * In addition, if the very first line has whitespace only, then
     * that line is removed. This just means that the {= delimiter
     * is followed by a newline.
     * 
     * @param code the code block to be trimmed
     * @return trimmed code block 
     */
    def static trimCodeBlock(String code) {
        var codeLines = code.split("\n")
        var String prefix = null
        var buffer = new StringBuilder()
        var first = true
        for (line : codeLines) {
            if (prefix === null) {
                if (line.trim.length > 0) {
                    // this is the first code line
                    
                    // find the index of the first code line
                    val characters = line.toCharArray()
                    var foundFirstCharacter = false
                    var int firstCharacter = 0
                    for (var i = 0; i < characters.length(); i++) {
                        if (!foundFirstCharacter && !Character.isWhitespace(characters.get(i))) {
                            foundFirstCharacter = true
                            firstCharacter = i
                        }
                    }

                    // extract the whitespace prefix
                    prefix = line.substring(0, firstCharacter)
                } else if(!first) {
                    // Do not remove blank lines. They throw off #line directives.
                    buffer.append(line)
                    buffer.append('\n')
                }
            }
            first = false

            // try to remove the prefix from all subsequent lines
            if (prefix !== null) {
                if (line.startsWith(prefix)) {
                    buffer.append(line.substring(prefix.length))
                    buffer.append('\n')
                } else {
                    buffer.append(line)
                    buffer.append('\n')
                }
            }
        }
        if (buffer.length > 1) {
        	buffer.deleteCharAt(buffer.length - 1) // remove the last newline
        } 
        buffer.toString
    }
    
    /**
     * Convert a time to its textual representation as it would
     * appear in LF code.
     * 
     * @param t The time to be converted
     * @return A textual representation
     */
    def static String toText(Time t) '''«t.interval» «t.unit.toString»'''
        
    /**
     * Convert a value to its textual representation as it would
     * appear in LF code.
     * 
     * @param v The value to be converted
     * @return A textual representation
     */
    def static String toText(Value v) {
        if (v.parameter !== null) {
            return v.parameter.name
        }
        if (v.time !== null) {
            return v.time.toText
        }
        if (v.literal !== null) {
            return v.literal
        }
        if (v.code !== null) {
            return v.code.toText
        }
        ""
    }
    
    /**
     * Return a string of the form either "name" or "container.name" depending
     * on in which form the variable reference was given.
     * @param v The variable reference.
     */
    def static toText(VarRef v) {
        if (v.container !== null) {
            '''«v.container.name».«v.variable.name»'''
        } else {
            '''«v.variable.name»'''
        }
    }
    
    /**
     * Convert an array specification to its textual representation as it would
     * appear in LF code.
     * 
     * @param spec The array spec to be converted
     * @return A textual representation
     */
    def static toText(ArraySpec spec) {
        if (spec !== null) {
            return (spec.ofVariableLength) ? "[]" : "[" + spec.length + "]"
        }
    }
    
    /**
     * Translate the given type into its textual representation, including
     * any array specifications.
     * @param type AST node to render as string.
     * @return Textual representation of the given argument.
     */
    def static toText(Type type) {
        if (type !== null) {
            val base = type.baseType
            val arr = (type.arraySpec !== null) ? type.arraySpec.toText : ""
            return base + arr
        }
        ""
    }
    
    /**
     * Translate the given type into its textual representation, but
     * do not append any array specifications.
     * @param type AST node to render as string.
     * @return Textual representation of the given argument.
     */
    def static baseType(Type type) {
        if (type !== null) {
            if (type.code !== null) {
                return toText(type.code)
            } else {
                if (type.isTime) {
                    return "time"
                } else {
                    var stars = ""
                    for (s : type.stars ?: emptyList) {
                        stars += s
                    }
                    return type.id + stars
                }
            }
        }
        ""
    }
        
    /**
     * Report whether the given literal is zero or not.
     * @param literalOrCode AST node to inspect.
     * @return True if the given literal denotes the constant `0`, false
     * otherwise.
     */
    def static boolean isZero(String literal) {
        try {
            if (literal !== null &&
                Integer.parseInt(literal) == 0) {
                return true
            }
        } catch (NumberFormatException e) {
            // Not an int.
        }
        return false
    }
    
    def static boolean isZero(Code code) {
        if (code !== null && code.toText.isZero) {
            return true
        }
        return false
    }
    
    /**
     * Report whether the given value is zero or not.
     * @param value AST node to inspect.
     * @return True if the given value denotes the constant `0`, false otherwise.
     */
    def static boolean isZero(Value value) {
        if (value.literal !== null) {
            return value.literal.isZero
        } else if (value.code !== null) {
            return value.code.isZero
        }
        return false
    }
    
    
    /**
     * Report whether the given string literal is an integer number or not.
     * @param literal AST node to inspect.
     * @return True if the given value is an integer, false otherwise.
     */
    def static boolean isInteger(String literal) {
        try {
            Integer.parseInt(literal)
        } catch (NumberFormatException e) {
            return false
        }
        return true
    }

	/**
     * Report whether the given code is an integer number or not.
     * @param code AST node to inspect.
     * @return True if the given code is an integer, false otherwise.
     */
	def static boolean isInteger(Code code) {
        return code.toText.isInteger
    }
    
    /**
     * Report whether the given value is an integer number or not.
     * @param value AST node to inspect.
     * @return True if the given value is an integer, false otherwise.
     */
    def static boolean isInteger(Value value) {
        if (value.literal !== null) {
            return value.literal.isInteger
        } else if (value.code !== null) {
            return value.code.isInteger
        }
        return false
    }
    
    /**
     * Report whether the given value denotes a valid time or not.
     * @param value AST node to inspect.
     * @return True if the argument denotes a valid time, false otherwise.
     */
    def static boolean isValidTime(Value value) {
        if (value !== null) {
            if (value.parameter !== null) {
                if (value.parameter.isOfTimeType) {
                    return true
                }
            } else if (value.time !== null) {
                return isValidTime(value.time)
            } else if (value.literal !== null) {
                if (value.literal.isZero) {
                    return true
                }
            } else if (value.code !== null) {
                if (value.code.isZero) {
                    return true
                }
            }
        }
        return false
    }

    /**
     * Report whether the given time denotes a valid time or not.
     * @param value AST node to inspect.
     * @return True if the argument denotes a valid time, false otherwise.
     */
    def static boolean isValidTime(Time t) {
        if (t !== null && t.unit != TimeUnit.NONE) {
            return true
        }
        return false
    }

	/**
     * Report whether the given parameter denotes time list, meaning it is a list
     * of which all elements are valid times.
     * @param value AST node to inspect.
     * @return True if the argument denotes a valid time list, false otherwise.
     */
	def static boolean isValidTimeList(Parameter p) {
        if (p !== null) {
            if (p.type !== null && p.type.isTime && p.type.arraySpec !== null) {
                return true
            } else if (p.init !== null && p.init.size > 1 && p.init.forall [
                it.isValidTime
            ]) {
                return true
            }
        }
        return true
    }

    /**
     * Report whether the given parameter has been declared a type or has been
     * inferred to be a type. Note that if the parameter was declared to be a
     * time, its initialization may still be faulty (assigning a value that is 
     * not actually a valid time).
     * @param A parameter
     * @return True if the argument denotes a time, false otherwise.
     */
    def static boolean isOfTimeType(Parameter p) {
        if (p !== null) {
            // Either the type has to be declared as a time.
            if (p.type !== null && p.type.isTime) {
                return true
            }
            // Or it has to be initialized as a proper time with units.
            if (p.init !== null && p.init.size == 1) {
                val time = p.init.get(0).time
                if (time !== null && time.unit != TimeUnit.NONE) {
                    return true
                }
            } 
            // In other words, one can write:
            // - `x:time(0)` -OR- 
            // - `x:(0 msec)`, `x:(0 sec)`, etc.     
        }
        return false
    }

    /**
     * Report whether the given state variable denotes a time or not.
     * @param A state variable
     * @return True if the argument denotes a time, false otherwise.
     */
    def static boolean isOfTimeType(StateVar s) {
        if (s !== null) {
            // Either the type has to be declared as a time.
            if (s.type !== null)
                return s.type.isTime
            // Or the it has to be initialized as a time except zero.
            if (s.init !== null && s.init.size == 1) {
                val init = s.init.get(0)
                if (init.isValidTime && !init.isZero)
                    return true
            }   
            // In other words, one can write:
            // - `x:time(0)` -OR- 
            // - `x:(0 msec)`, `x:(0 sec)`, etc. -OR-
            // - `x:(p)` where p is defined as above.
        }
        return false
    }
    
    /**
	 * Assuming that the given parameter is of time type, return
	 * its initial value.
	 * @param p The AST node to inspect.
	 * @return A time value based on the given parameter's initial value.
	 */    
    def static TimeValue getInitialTimeValue(Parameter p) {
        if (p !== null && p.isOfTimeType) {
            val init = p.init.get(0)
            if (init.time !== null) {
                return new TimeValue(init.time.interval, init.time.unit)
            } else if (init.parameter !== null) {
                // Parameter value refers to another parameter.
                return getInitialTimeValue(init.parameter) 
            } else {
                return new TimeValue(0, TimeUnit.NONE)
            }
        }
    }
    
    /**
	 * Assuming that the given value denotes a valid time, return a time value.
	 * @param p The AST node to inspect.
	 * @return A time value based on the given parameter's initial value.
	 */        
    def static TimeValue getTimeValue(Value v) {
        if (v.parameter !== null) {
            return ASTUtils.getInitialTimeValue(v.parameter)
        } else if (v.time !== null) {
            return new TimeValue(v.time.interval, v.time.unit)
        } else {
            return new TimeValue(0, TimeUnit.NONE)
        }
    }
        
    /**
     * Return the width of the port reference if it can be determined
     * and otherwise return -1.  The width can be determined if the
     * port is not a multiport in a bank of reactors (the width will 1)
     * or if the width of the multiport and/or the bank is given by a
     * literal constant.
     * @param reference A reference to a port.
     * @return The width of a port or -1 if it cannot be determined.
     */
    def static int multiportWidth(VarRef reference) {
        if (reference.variable instanceof Port) {
            var bankWidth = 1
            if (reference.container !== null) {
                bankWidth = width(reference.container.widthSpec)
                if (bankWidth < 0) return -1
            }
            val portWidth = width((reference.variable as Port).widthSpec)
            if (portWidth > 0) return portWidth * bankWidth
        }
        return -1
    }    

    /**
     * Given the specification of the width of either a bank of reactors
     * or a multiport, return the width if it can be determined and otherwise
     * return -1. The width can be determined if it is given by one or more
     * literal constants or if the widthSpec is null (it is not a multiport
     * or reactor bank).
     * @param widthSpec The width specification.
     * @return The width or -1 if it cannot be determined.
     */
    def static int width(WidthSpec widthSpec) {
        if (widthSpec === null) return 1
        var result = 0
        if (widthSpec.ofVariableLength) {
            return -1
        }
        for (term : widthSpec.terms) {
            if (term.parameter === null) {
                result += term.width
            } else {
                return -1
            }
        }
        return result
    }
    
    /**
     * Calculate the width of a port reference in a connection.
     * The width will be the product of the bank width and the multiport width,
     * or 1 if the port is not in a bank and is not a multiport.
     * This throws an exception if the width depends on a parameter value.
     * If the width depends on a parameter value, then this method
     * will need to determine that parameter for each instance, not
     * just class definition of the containing reactor.
     */
    def static int portWidth(VarRef port, Connection c) {
        val result = port.multiportWidth
        if (result < 0) {
            // The port may be in a bank that has variable width,
            // in which case, we attempt to infer its width.
            // Specifically, this supports 'after' in multiport and reactor bank connections.
            if (port.container !== null && port.container.widthSpec !== null && port.container.widthSpec.isOfVariableLength) {
                // This could be a bank of delays.
                var leftWidth = 0
                var rightWidth = 0
                var leftOrRight = 0
                for (leftPort : c.leftPorts) {
                    if (leftPort === port) {
                        if (leftOrRight !== 0) {
                            throw new Exception("Multiple ports with variable width on a connection.")
                        }
                        // Indicate that the port is on the left.
                        leftOrRight = -1
                    } else {
                        leftWidth += portWidth(leftPort, c)
                    }
                }
                for (rightPort : c.rightPorts) {
                    if (rightPort === port) {
                        if (leftOrRight !== 0) {
                            throw new Exception("Multiple ports with variable width on a connection.")
                        }
                        // Indicate that the port is on the right.
                        leftOrRight = 1
                    } else {
                        rightWidth += portWidth(rightPort, c)
                    }
                }
                if (leftOrRight < 0) {
                    return rightWidth - leftWidth
                } else if (leftOrRight > 0) {
                    return leftWidth - rightWidth
                }
            }
            
            throw new Exception("Cannot determine port width. Only multiport widths with literal integer values are supported for now.")
        }
        return result
    }
    
    /**
     * Given an instantiation of a reactor or bank of reactors, return
     * the width. This will be 1 if this is not a reactor bank. Otherwise,
     * this will attempt to determine the width. If the width is declared
     * as a literal constant, it will return that constant. If the width
     * is specified as a reference to a parameter, this will throw an
     * exception. If the width is variable, this will find
     * connections in the enclosing reactor and attempt to infer the
     * width. If the width cannot be determined, it will throw an exception.
     * @param instantiation A reactor instantiation.
     * @return The width, if it can be determined.
     */
    def static int widthSpecification(Instantiation instantiation) {
        if (instantiation.widthSpec === null) return 1
        if (instantiation.widthSpec.ofVariableLength) {
            // Attempt to infer the width.
            for (c : (instantiation.eContainer as Reactor).connections) {
                var leftWidth = 0
                var rightWidth = 0
                var leftOrRight = 0
                for (leftPort : c.leftPorts) {
                    if (leftPort.container === instantiation) {
                        if (leftOrRight !== 0) {
                            throw new Exception("Multiple ports with variable width on a connection.")
                        }
                        // Indicate that the port is on the left.
                        leftOrRight = -1
                    } else {
                        leftWidth += portWidth(leftPort, c)
                    }
                }
                for (rightPort : c.rightPorts) {
                    if (rightPort.container === instantiation) {
                        if (leftOrRight !== 0) {
                            throw new Exception("Multiple ports with variable width on a connection.")
                        }
                        // Indicate that the port is on the right.
                        leftOrRight = 1
                    } else {
                        rightWidth += portWidth(rightPort, c)
                    }
                }
                if (leftOrRight < 0) {
                    return rightWidth - leftWidth
                } else if (leftOrRight > 0) {
                    return leftWidth - rightWidth
                }
            }
            // A connection was not found with the instantition.
            throw new Exception("Cannot determine width.")
        }
        var result = 0
        for (term : instantiation.widthSpec.terms) {
            if (term.parameter === null) {
                result += term.width
            } else {
                throw new Exception("Cannot determine width for the class because it depends on parameter "
                    + term.parameter.name
                )
            }
        }
        return result
    }

    /**
     * Report whether a state variable has been initialized or not.
     * @param v The state variable to be checked.
     * @return True if the variable was initialized, false otherwise.
     */
    def static boolean isInitialized(StateVar v) {
        if (v !== null && v.parens.size == 2) {
            return true
        }
        return false
    }
        
    /**
     * Report whether the given time state variable is initialized using a 
     * parameter or not.
     * @param s A state variable.
     * @return True if the argument is initialized using a parameter, false 
     * otherwise.
     */
    def static boolean isParameterized(StateVar s) {
        if (s.init !== null && s.init.exists[it.parameter !== null]) {
            return true
        }
        return false
    }
    
    /**
     * Given an initialization list, return an inferred type. Only two types
     * can be inferred: "time" and "timeList". Return the "undefined" type if
     * neither can be inferred.
     * @param initList A list of values used to initialize a parameter or
     * state variable.
     * @return The inferred type, or "undefined" if none could be inferred.
     */    
    protected static def InferredType getInferredType(EList<Value> initList) {
        if (initList.size == 1) {
        	// If there is a single element in the list, and it is a proper
        	// time value with units, we infer the type "time".
            val init = initList.get(0)
            if (init.parameter !== null) {
                return init.parameter.getInferredType
            } else if (init.isValidTime && !init.isZero) {
                return InferredType.time;
            }
        } else if (initList.size > 1) {
			// If there are multiple elements in the list, and there is at
			// least one proper time value with units, and all other elements
			// are valid times (including zero without units), we infer the
			// type "time list".
            var allValidTime = true
            var foundNonZero = false

            for (init : initList) {
                if (!init.isValidTime) {
                    allValidTime = false;
                } 
                if (!init.isZero) {
                    foundNonZero = true
                }
            }

            if (allValidTime && foundNonZero) {
                // Conservatively, no bounds are inferred; the returned type 
                // is a variable-size list.
				return InferredType.timeList()
            }
        }
        return InferredType.undefined
    }
    
    /**
     * Given a parameter, return an inferred type. Only two types can be
     * inferred: "time" and "timeList". Return the "undefined" type if
     * neither can be inferred.
     * @param p A parameter to infer the type of. 
     * @return The inferred type, or "undefined" if none could be inferred.
     */    
    def static InferredType getInferredType(Parameter p) {
        if (p !== null) {
            if (p.type !== null) {
                return InferredType.fromAST(p.type)
            } else {
                return p.init.inferredType
            }
        }
        return InferredType.undefined
    }
    
    /**
	 * Given a state variable, return an inferred type. Only two types can be
	 * inferred: "time" and "timeList". Return the "undefined" type if
	 * neither can be inferred.
     * @param s A state variable to infer the type of. 
     * @return The inferred type, or "undefined" if none could be inferred.
     */    
    def static InferredType getInferredType(StateVar s) {
        if (s !== null) {
            if (s.type !== null) {
                return InferredType.fromAST(s.type)
            }
            if (s.init !== null) {
                return s.init.inferredType
            }
        }
        return InferredType.undefined
    }
    
    /**
     * Construct an inferred type from an "action" AST node based
     * on its declared type. If no type is declared, return the "undefined"
     * type.
     * @param a An action to construct an inferred type object for.
     * @return The inferred type, or "undefined" if none was declared.
     */
    def static InferredType getInferredType(Action a) {
        if (a !== null && a.type !== null) {
            return InferredType.fromAST(a.type)
        }
        return InferredType.undefined
    }

	/**
     * Construct an inferred type from a "port" AST node based on its declared
     * type. If no type is declared, return the "undefined" type.
     * @param p A port to construct an inferred type object for.
     * @return The inferred type, or "undefined" if none was declared.
     */    
    def static InferredType getInferredType(Port p) {
        if (p !== null && p.type !== null) {
            return InferredType.fromAST(p.type)
        }
        return InferredType.undefined
    }

    /**
     * Check if the reactor class uses generics
     * @param r the reactor to check 
     * @true true if the reactor uses generics
     */
    def static boolean isGeneric(Reactor r) {
        var Reactor defn = r
        
        return defn?.typeParms.length != 0;
    }
    
    
    def static Reactor toDefinition(ReactorDecl r) {
        if (r === null)
            return null
        if (r instanceof Reactor) {
            return r
        } else if (r instanceof ImportedReactor) {
            return r.reactorClass
        }
    }
    /**
     * Retrieve a specific annotation in a JavaDoc style comment associated with the given model element in the AST.
     * 
     * This will look for a JavaDoc style comment. If one is found, it searches for the given annotation `key`.
     * and extracts any string that follows the annotation marker.  
     * 
     * @param object the AST model element to search a comment for
     * @param key the specific annotation key to be extracted
     * @return `null` if no JavaDoc style comment was found or if it does not contain the given key.
     *     The string immediately following the annotation marker otherwise.
     */
    def static String findAnnotationInComments(EObject object, String key) {
        if (object.eResource instanceof XtextResource) {
            val compNode = NodeModelUtils.findActualNodeFor(object)
            if (compNode !== null) {
                var node = compNode.firstChild
                while (node instanceof CompositeNode) {
                    node = node.firstChild
                }
                while (node instanceof HiddenLeafNode) { // Only comments preceding start of element
                    val rule = node.grammarElement
                    if (rule instanceof TerminalRule) {
                        var String line;
                        if ("SL_COMMENT".equals(rule.name)) {
                            if (node.text.contains(key)) {
                                line = node.text
                            }
                        } else if ("ML_COMMENT".equals(rule.name)) {
                            var found = false
                            for (str : node.text.split("\n")) {
                                if (!found && str.contains(key)) {
                                    line = str
                                }
                            }
                            // This is shorter but causes a warning:
                            //line = node.text.split("\n").filterNull.findFirst[it.contains(key)]
                        }
                        if (line !== null) {
                            var value = line.substring(line.indexOf(key) + key.length).trim()
                            if (value.contains("*")) { // in case of single line block comment (e.g. /** @anno 1503 */)
                                value = value.substring(0, value.indexOf("*")).trim()
                            }
                            return value
                        }
                    }
                    node = node.nextSibling
                }
            }
        }
        return null
    }
    
    /**
     * Remove quotation marks surrounding the specified string.
     */
    static def withoutQuotes(String s) {
        var result = s
        if (s.startsWith("\"") || s.startsWith("\'")) {
            result = s.substring(1)
        }
        if (result.endsWith("\"") || result.endsWith("\'")) {
            result = result.substring(0, result.length - 1)
        }
        result
    }
    
    /**
     * Search for an `@label` annotation for a given reaction.
     * 
     * @param n the reaction for which the label should be searched
     * @return The annotated string if an `@label` annotation was found. `null` otherwise.
     */
    def static String label(Reaction n) {
        return n.findAnnotationInComments("@label")
    }
}
