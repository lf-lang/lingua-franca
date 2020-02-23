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

package org.icyphy.generator

import java.util.HashSet
import org.icyphy.linguaFranca.Variable

/** Instance of a trigger (port, action, or timer).
 * 
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class TriggerInstance<T extends Variable> extends NamedInstance<Variable> {

    /** Construct a new instance with the specified definition
     *  and parent. E.g., for a action instance, the definition
     *  is Action, and for a port instance, it is Port. These are
     *  nodes in the AST. This is protected because only subclasses
     *  should be constructed.
     *  @param definition The definition in the AST for this instance.
     *  @param parent The reactor instance that creates this instance.
     */
    protected new(Variable definition, ReactorInstance parent) {
        super(definition, parent)
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /** Reaction instances that are triggered by this trigger. */
    public var dependentReactions = new HashSet<ReactionInstance>();

    /** Reaction instances that may send outputs via this port. */
    public var dependsOnReactions = new HashSet<ReactionInstance>(); // FIXME: Perhaps better to use a TreeSet here

    /////////////////////////////////////////////
    //// Public Methods

    /** Return the name of this timer. 
     *  @return The name of this timer.
     */
    override getName() {
        this.definition.name
    }

    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    override ReactorInstance main() {
        this.parent.main
    }   
}
