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
import org.icyphy.linguaFranca.Delay
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.LiteralOrCode
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.StateVar
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Type
import org.icyphy.linguaFranca.Parameter

/**
 * A helper class for modifying and analyzing the AST.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class ASTUtils {
    
    /**
     * An instance of a Lingua Franca factory for creating new AST nodes.
     */
    public static val factory = LinguaFrancaFactory.eINSTANCE
    
    /**
     * An instance of a Lingua Franca code generator.
     */
    GeneratorBase generator
    
    /**
     * Create a new instance.
     * @param generator An instance of a Lingua Franca code generator.
     */
    new(GeneratorBase generator) {
        this.generator = generator
    }
    
    /**
     * Take a connection and replace it with an action and two reactions
     * that implement a delayed transfer between the end points of the 
     * given connection.
     * @param connection The connection to replace.
     * @param delay The delay associated with the connection.
     */
    def void desugarDelay(Connection connection, Delay delay) {
        val factory = LinguaFrancaFactory.eINSTANCE
        val type = duplicateType((connection.rightPort.variable as Port).type)
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
        action.minDelay = connection.delay.time
        
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
        // FIXME: Why the extra semicolon?
        r1.code = factory.createCode()
        r1.code.tokens.add(this.generator.generateDelayBody(action, inRef))

        // Configure the second reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        // FIXME: Why the extra semicolon?
        r2.code = factory.createCode()
        r2.code.tokens.add(this.generator.generateForwardBody(action, outRef))

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
     * @param rightFederate The destination federate.
     */
    def void makeCommunication(
        Connection connection, 
        FederateInstance leftFederate,
        FederateInstance rightFederate
    ) {
        val factory = LinguaFrancaFactory.eINSTANCE
        var type = duplicateType((connection.rightPort.variable as Port).type)
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
        r1.code.tokens.add(generator.generateNetworkSenderBody(
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            toText(type)
        ))

        // Configure the receiving reaction.
        r2.triggers.add(triggerRef)
        r2.effects.add(outRef)
        r2.code.tokens.add(generator.generateNetworkReceiverBody(
            action,
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            toText(type)
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
    def getUniqueIdentifier(Reactor reactor, String name) {
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
     * Create a new type based on the given type.
     * @param type The type to duplicate.
     * @return A deep copy of the given type.
     */
    private def duplicateType(Type type) {
        if (type !== null) {
            val newType = factory.createType
            
            // Set the type based on the argument type.
            if (type.code !== null) {
                newType.code = factory.createCode
                type.code.tokens?.forEach[newType.code.tokens.add(it)]
            } else {
                newType.id = type.id
                type.stars?.forEach[newType.stars.add(it)]
                if (type.arraySpec !== null) {
                    newType.arraySpec = factory.createArraySpec
                    newType.arraySpec.ofVariableLength = type.arraySpec.
                        ofVariableLength
                    newType.arraySpec.length = type.arraySpec.length
                }
            }
            return newType
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
                val str = builder.toString.trim
                return str.substring(2, str.length - 2).trim    
            }
        }
        return ""
    }
    
    /**
     * Translate the given type into its textual representation.
     * @param type AST node to render as string.
     * @return Textual representation of the given argument.
     */
    def static toText(Type type) {
        if (type !== null) {
            if (type.code !== null) {
                return toText(type.code)
            } else {
                var stars = ""
                for (s : type.stars ?: emptyList) {
                    stars += s
                }
                var arraySpec = ""
                if (type.arraySpec !== null) {
                    arraySpec = (type.arraySpec.ofVariableLength)? 
                        "[]" : 
                        "[" + type.arraySpec.length + "]"  
                }
                return type.id + stars + arraySpec
            }
        }
        ""
    }
    
    /**
     * Translate the given literal or code into a textual representation.
     * @param literalOrCode AST node to render as string.
     * @return Textual representation of the given argument.
     */
    def static String toText(LiteralOrCode literalOrCode) {
        if (literalOrCode === null) {
            return ""
        } else if (literalOrCode.literal !== null) {
            return literalOrCode.literal
        } else {
            return toText(literalOrCode.code)
        }
    }
    
    /**
     * Report whether the given literal is zero or not.
     * @param literalOrCode AST node to inspect.
     * @return True if the given literal denotes the constant `0`, false
     * otherwise.
     */
    def static boolean isZero(LiteralOrCode literalOrCode) {
        try {
            if (literalOrCode !== null &&
                Integer.parseInt(literalOrCode.toText.trim) == 0) {
                return true
            }
        } catch (NumberFormatException e) {
            // NaN
        }
        return false
    }
    
    /**
     * Report whether the given time or value denotes a valid time or not.
     * @param tv A time or value.
     * @return True if the argument denotes a valid time, false otherwise.
     */
    def static boolean isValidTime(TimeOrValue tv) {
        if (tv !== null &&
            ((tv.time == 0 || tv.unit != TimeUnit.NONE) ||
                (tv.parameter !== null && tv.parameter.isOfTimeType))) {
            return true
        }
        return false
    }
    
    /**
     * Given a state variable, return the AST node that denotes its type.
     * Caution: if the given state variable is of time type, then this method 
     * returns null.
     * @param s A state variable.
     * @return The type associated with the argument, or null if denotes a time.
     */
    def static Type getValueType(StateVar s) {
        if (s !== null) {
            if (s.type !== null) {
                return s.type
            } else if (s.init !== null && s.init.size == 1) {
                // If this parameter is initialized using a list,
                // then it needs a type.
                val parm = s.init.get(0).parameter
                if (parm !== null)
                    return parm.type
            }
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
        if (s.init !== null && s.init.exists[it instanceof Parameter]) {
            return true
        }
        return false
    }
}
