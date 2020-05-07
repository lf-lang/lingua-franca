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

import java.util.HashSet
import java.util.LinkedList
import java.util.List
import java.util.Set
import org.eclipse.emf.common.util.EList
import org.eclipse.xtext.nodemodel.ILeafNode
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.generator.FederateInstance
import org.icyphy.generator.GeneratorBase
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.ArraySpec
import org.icyphy.linguaFranca.Code
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.TypeParm
import org.eclipse.emf.ecore.resource.Resource
import java.util.HashMap

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
        val newConnections = new HashMap<Reactor, List<Connection>>()
        val delayInstances = new HashMap<Reactor, List<Instantiation>>()
        val delayClasses = new HashSet<Reactor>()
        
        // Iterate over the connections in the tree.
        for (connection : resource.allContents.toIterable.filter(Connection)) {
            if (connection.delay !== null) {
                val parent = connection.eContainer as Reactor
                val type = (connection.rightPort.variable as Port).type
                val delayClass = getDelayClass(type, delayClasses, generator)
                val generic = generator.supportsGenerics ? 
                    InferredType.fromAST(type).toText : ""
                val delayInstance = getDelayInstance(delayClass,
                    connection.delay, generic)

                // Stage the new connections for insertion into the tree.
                var connections = newConnections.get(parent)
                if (connection !== null) connections = new LinkedList()
                connections.addAll(connection.rerouteViaDelay(delayInstance))
                newConnections.put(parent, connections)

                // Stage the original connection for deletion from the tree.
                oldConnections.add(connection)

                // Stage the newly created delay reactor instance for insertion
                var instances = (delayInstances.get(parent) ?: emptyList)
                if (instances !== null) instances = new LinkedList()
                instances.addAll(delayInstance)
                delayInstances.put(parent, instances)
            }
        }
        // Remove old connections; insert new ones.
        oldConnections.forEach [ connection |
            (connection.eContainer as Reactor).connections.remove(connection)
        ]
        newConnections.forEach [ reactor, connections |
            reactor.connections.addAll(connections)
        ]
        // Also add class definitions of the created delay reactor(s).
        delayClasses.forEach[reactor|resource.contents.add(reactor)]
        // Finally, insert the instances and, before doing so, assign them a unique name.
        delayInstances.forEach [ reactor, instantiations |
            instantiations.forEach [ instantiation |
                instantiation.name = reactor.getUniqueIdentifier("delay");
                reactor.instantiations.add(instantiation)
            ]
        ]
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
        
        val delayClass = delayInstance.reactorClass
        
        // Establish references to the involved ports.
        input.container = delayInstance
        input.variable = delayClass.inputs.get(0)
        output.container = delayInstance
        output.variable = delayClass.outputs.get(0)
        upstream.leftPort = connection.leftPort
        upstream.rightPort = input
        downstream.leftPort = output
        downstream.rightPort = connection.rightPort

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
     * performed in this method to avoid causing concurrent modification
     * exceptions. 
     * @param delayClass The class to create an instantiation for
     * @param value A time interval corresponding to the desired delay
     * @param generic A string that denotes the appropriate type parameter, 
     * which should be null or empty if the target does not support generics. 
     */
    private static def Instantiation getDelayInstance(Reactor delayClass, 
            Value time, String generic) {
        val delayInstance = factory.createInstantiation
        delayInstance.reactorClass = delayClass
        if (!generic.isNullOrEmpty) {
            val typeParm = factory.createTypeParm
            typeParm.literal = generic
            delayInstance.typeParms.add(typeParm)
            
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
     * @param generator A code generator.
     */
    private static def Reactor getDelayClass(Type type, 
            Set<Reactor> generatedClasses, GeneratorBase generator) {
        
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

        // Configure the first reaction.
        r1.triggers.add(inRef)
        r1.effects.add(effectRef)
        r1.code = factory.createCode()
        r1.code.tokens.add(generator.generateDelayBody(action, inRef))
    
        // Configure the second reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        r2.code = factory.createCode()
        r2.code.tokens.add(generator.generateForwardBody(action, outRef))    
    
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
        
        return delayClass
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
        var type = (connection.rightPort.variable as Port).type.copy
        val action = factory.createAction
        val triggerRef = factory.createVarRef
        val effectRef = factory.createVarRef
        val inRef = factory.createVarRef
        val outRef = factory.createVarRef
        val parent = (connection.eContainer as Reactor)
        val r1 = factory.createReaction
        val r2 = factory.createReaction

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
        inRef.container = connection.leftPort.container
        inRef.variable = connection.leftPort.variable
        outRef.container = connection.rightPort.container
        outRef.variable = connection.rightPort.variable

        // Add the action to the reactor.
        parent.actions.add(action)

        // Configure the sending reaction.
        r1.triggers.add(inRef)
        r1.effects.add(effectRef)
        r1.code = factory.createCode()
        r1.code.tokens.add(generator.generateNetworkSenderBody(
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            action.inferredType
        ))

        // Configure the receiving reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        r2.code = factory.createCode()
        r2.code.tokens.add(generator.generateNetworkReceiverBody(
            action,
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            action.inferredType
        ))

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
        val vars = new HashSet<String>();
        reactor.actions.forEach[it | vars.add(it.name)]
        reactor.timers.forEach[it | vars.add(it.name)]
        reactor.parameters.forEach[it | vars.add(it.name)]
        reactor.inputs.forEach[it | vars.add(it.name)]
        reactor.outputs.forEach[it | vars.add(it.name)]
        reactor.stateVars.forEach[it | vars.add(it.name)]
        reactor.instantiations.forEach[it | vars.add(it.name)]
        
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
                original.code.tokens.forEach[clone.code.tokens.add(it)]
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
                original.code.tokens.forEach[clone.code.tokens.add(it)]
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
                original.code.tokens?.forEach[clone.code.tokens.add(it)]
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
                // remove the code delimiters
                str = str.substring(2, str.length - 2)
                if (str.split('\n').length > 1) {
                    // multi line code
                    return str.trimCodeBlock
                } else {
                    // single line code
                    return str.trim
                }    
            } else {
                // Code must have been added as a simple string.
                val builder = new StringBuilder(Math.max(code.tokens.length, 1))
                for (token : code.tokens) {
                    builder.append(token)
                }
                return builder.toString
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
	 * First this removes any lines only containing whitespaces in the beginning
	 * of the block. It considers the first line containing non-whitespace 
	 * characters as the first code line. The leading whitespaces of this first
	 * code line are considered as a common prefix across all code lines. If the
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
     * @param code the code block to be trimmed
     * @return trimmed code block 
     */
    def static trimCodeBlock(String code) {
        var codeLines = code.split("\n")
        var String prefix = null
        var buffer = new StringBuilder()
        for (line : codeLines) {
            if (prefix === null) {
                // skip any lines that only contain whitespaces
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
                }
            }

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
        if (buffer.length > 1)
        	buffer.deleteCharAt(buffer.length - 1) // remove the last newline 
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
    def static isGeneric(Reactor r) {
        return r.typeParms.length != 0;
    }
}
