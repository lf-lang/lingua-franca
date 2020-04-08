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
import org.eclipse.xtext.nodemodel.ILeafNode
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.generator.FederateInstance
import org.icyphy.generator.GeneratorBase
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Code
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.ArraySpec
import org.icyphy.linguaFranca.Value
import org.eclipse.emf.common.util.EList
import org.icyphy.linguaFranca.Action

/**
 * A helper class for modifying and analyzing the AST.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 */
class ASTUtils {
    
    /**
     * An instance of a Lingua Franca factory for creating new AST nodes.
     */
    public static val factory = LinguaFrancaFactory.eINSTANCE
    
    
    /**
     * Take a connection and replace it with an action and two reactions
     * that implement a delayed transfer between the end points of the 
     * given connection.
     * @param connection The connection to replace.
     * @param delay The delay associated with the connection.
     */
    static def void desugarDelay(Connection connection, GeneratorBase generator) {
        val type = (connection.rightPort.variable as Port).type.copy
        val action = factory.createAction
        val triggerRef = factory.createVarRef
        val effectRef = factory.createVarRef
        val inRef = factory.createVarRef
        val outRef = factory.createVarRef
        val parent = (connection.eContainer as Reactor)
        val r1 = factory.createReaction
        val r2 = factory.createReaction

        // Name the newly created action; set its delay and type.
        action.name = getUniqueIdentifier(parent, "delay")
        action.minDelay = factory.createValue
        action.minDelay = connection.delay.copy
         
        action.type = type
        action.origin = ActionOrigin.LOGICAL

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

        // Add the reactions to the parent.
        // These need to go in the opposite order in case
        // a new input arrives at the same time the delayed
        // output is delivered!
        parent.reactions.add(r2)
        parent.reactions.add(r1)
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
        // FIXME: Handle connection delay. E.g.:  
        // action.minTime = connection.delay.time
        action.type = type
        // FIXME: For now, only handling physical connections.
        action.origin = ActionOrigin.PHYSICAL
        
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
     * Create a new type based on the given type.
     * @param type The type to duplicate.
     * @return A deep copy of the given type.
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
    
    /**
     * Intelligently trim the whitespaces in a code block.
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
        buffer.deleteCharAt(buffer.length - 1) // remove the last newline 
        buffer.toString
    }

    @Deprecated // FIXME: Delete. Use totext(Time) or getTargetTime(Time) of the code generator
    def static String toText(Time t, GeneratorBase generator) {
        if (generator !== null) {
            generator.timeInTargetLanguage(new TimeValue(t.interval, t.unit))  
        } else {
            t.interval + " " + t.unit.toString
        }
    }
    
    /**
     * Convert a time to its textual representation as it would
     * appear in LF code.
     * 
     * @param t The time to be converted
     * @return A textual representation
     */
    def static String toText(Time t) '''«t.interval» «t.unit.toString»'''
    
//    @Deprecated // FIXME: Delete. Use totext(Value), getTargetTime(Value), or getTargetValue(Value) of the code generator
//    def static String toText(Value v, GeneratorBase generator) {
//        if (v.parameter !== null) {
//            return v.parameter.name
//        }
//        if (v.time !== null) {
//            return v.time.toText(generator)
//        }
//        if (v.literal !== null) {
//            return v.literal
//        }
//        if (v.code !== null) {
//            return v.code.toText
//        }
//        ""
//    }
    
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
            // NaN
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
     * Report whether the given parameter, time, or value denotes a valid time 
     * or not.
     * @param tv A time or value.
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

    def static boolean isValidTime(Time t) {
        if (t !== null && t.unit != TimeUnit.NONE) {
            return true
        }
        return false
    }

    def static boolean isValidTimeList(Parameter p) { // FIXME: write a single function for both StateVar and Parameter?
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

    def static boolean isValidTimeList(StateVar s) {
        if (s !== null) {
            if (s.type !== null && s.type.isTime && s.type.arraySpec !== null) {
              // FIXME  
            }
        }
        return false
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
        
    def static TimeValue getTimeValue(Parameter p) {
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
     * Check if a state variable is initialized or not.
     * 
     * @param v The state variable to be checked
     * @return True if the variable was initialized
     */
    def static boolean isInitialized(StateVar v) {
        if (v !== null && v.parens.size == 2) {
            return true
        }
        return false
    }
    
    def static TimeValue getTimeValue(Value v) {
        if (v.parameter !== null) {
            return ASTUtils.getTimeValue(v.parameter)
        } else if (v.time !== null) {
            return new TimeValue(v.time.interval, v.time.unit)
        } else {
            return new TimeValue(0, TimeUnit.NONE)
        }
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
        
    protected static def InferredType getInferredType(EList<Value> initList) {
        if (initList.size == 1) {
            val init = initList.get(0)
            if (init.parameter !== null) {
                return init.parameter.getInferredType
            } else if (init.isValidTime && !init.isZero) {
                return InferredType.time;
            }
        } else if (initList.size > 1) {
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
                // FIXME: fixed size list or variable sized list

                // return a variable size time list
                return InferredType.timeList()
            }
        }
        return InferredType.undefined
    }
    
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
    
    def static InferredType getInferredType(Action a) {
        if (a !== null && a.type !== null) {
            return InferredType.fromAST(a.type)
        }
        return InferredType.undefined
    }
    
    def static InferredType getInferredType(Port p) {
        if (p !== null && p.type !== null) {
            return InferredType.fromAST(p.type)
        }
        return InferredType.undefined
    }
}
