/** Instance of a trigger (port, action, or timer). */

/*************
Copyright (c) 2019, The University of California at Berkeley.

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

import java.util.LinkedHashSet
import org.eclipse.xtend.lib.annotations.Accessors
import org.lflang.lf.TriggerRef
import org.lflang.lf.Variable
import org.lflang.lf.impl.VariableImpl

/** Instance of a trigger (port, action, or timer).
 * 
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class TriggerInstance<T extends Variable> extends NamedInstance<T> {
    
    /**
     * Special builtin trigger types.
     */
    enum BuiltinTrigger {
        STARTUP, SHUTDOWN
    }
    
    /** Construct a new instance with the specified definition
     *  and parent. E.g., for a action instance, the definition
     *  is Action, and for a port instance, it is Port. These are
     *  nodes in the AST. This is protected because only subclasses
     *  should be constructed.
     *  @param definition The definition in the AST for this instance.
     *  @param parent The reactor instance that creates this instance.
     */
    protected new(T definition, ReactorInstance parent) {
        super(definition, parent)
    }
    
    /**
     * Construct a new instance for a special builtin trigger.
     * This constructor must be used with the Variable or BuiltinTriggerVariable as generic type T.
     * 
     * @param type The builtin trigger type.
     * @param type The actual trigger definition.
     * @param parent The reactor instance that creates this instance.
     */
    package new(BuiltinTrigger type, TriggerRef trigger, ReactorInstance parent) {
        super(new BuiltinTriggerVariable(type, trigger) as T, parent)
        this.builtinTriggerType = type
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /** Reaction instances that are triggered by this trigger. */
    public var dependentReactions = new LinkedHashSet<ReactionInstance>();

    /** Reaction instances that may send outputs via this port. */
    public var dependsOnReactions = new LinkedHashSet<ReactionInstance>(); // FIXME: Perhaps better to use a TreeSet here

    /////////////////////////////////////////////
    //// Public Methods

    /** 
     * Return the name of this trigger. 
     * @return The name of this trigger.
     */
    override getName() {
        this.definition.name
    }

    /**
     * Return true if this trigger is "shutdown"./
     */
    def isShutdown() {
        return builtinTriggerType === BuiltinTrigger.SHUTDOWN
    }

    /**
     * Return true if this trigger is "startup"./
     */
    def isStartup() {
        return builtinTriggerType === BuiltinTrigger.STARTUP
    }

    /**
     * Return true if this trigger is a builtin one.
     */
    def isBuiltinTrigger() {
        return builtinTriggerType !== null
    }

    /**
     * {@inheritDoc}
     */
    override ReactorInstance root() {
        this.parent.root()
    }
    
    /**
     * Return true if this trigger is "startup".
     */

    /////////////////////////////////////////////
    //// Protected Fields
    
    @Accessors(PUBLIC_GETTER)
    protected var BuiltinTrigger builtinTriggerType = null;
    
    /////////////////////////////////////////////
    //// Special class for builtin triggers

    /**
     * This class allows to have BuiltinTriggers represented by a Variable type.
     */
    static class BuiltinTriggerVariable extends VariableImpl {
        /** The builtin trigger type represented by this variable. */
        public final BuiltinTrigger type
        /** The actual TriggerRef definition in the AST. */
        public final TriggerRef definition
        
        new(BuiltinTrigger type, TriggerRef trigger) {
            this.type = type
            this.definition = trigger
        }
        
        override getName() {
            this.type.name.toLowerCase
        }
        
        override setName(String newName) {
            throw new UnsupportedOperationException(this.class.simpleName + " has an immutable name.")
        }
    }
}
