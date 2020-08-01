/* Instantiation graph of a reactor program. */

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

package org.icyphy.graph

import com.google.common.collect.HashMultimap
import java.util.Set
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.ReactorDecl

import static extension org.icyphy.ASTUtils.*

/**
 * A graph with vertices that are instantiations and edges that denote
 * dependencies between them.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class InstantiationGraph extends PrecedenceGraph<Reactor> {

    /**
     * A mapping from reactors to the sites of their instantiation.
     */
    protected val reactorToInstantiation = HashMultimap.<Reactor, Instantiation>create
    
    /**
     * A mapping from reactor classes to their declarations.
     */
    protected val reactorToDecl = HashMultimap.<Reactor, ReactorDecl>create
    
    /**
     * Return the instantiations that point to a given reactor definition.
     */
    def Set<Instantiation> getInstantiations(Reactor definition) {
        return this.reactorToInstantiation.get(definition)
    }
    
    /**
     * Return the instantiations that point to a given reactor definition.
     */
    def Set<ReactorDecl> getDeclarations(Reactor definition) {
        return this.reactorToDecl.get(definition)
    }
    
    /**
     * Construct a reaction graph based on the given AST and run Tarjan's
     * algorithm to detect cyclic dependencies between reactions. It is
     * assumed that no instantiation cycles are present in the program.
     * Checks for instantiation cycles thus must be carried out prior to  
     * constructing the reaction graph.
     * @param model The AST.
     */
    new (Model model, boolean detectCycles) {
        for (r : model.reactors) {
            for (i : r.instantiations) {
                i.collectDeclarations
            }
        }
        detectCycles? this.detectCycles()
    }
    
    /**
     * Traverse the AST and build this precedence graph relating the
     * encountered instantiations. Also map each reactor to all
     * declarations associated with it and each reactor to the sites of
     * its instantiations.
     * @param instantiation
     * @param graph
     */
    private def void collectDeclarations(Instantiation instantiation) {
        val decl = instantiation.reactorClass
        val reactor = decl.toDefinition
        val container = instantiation.eContainer as Reactor
        
        this.reactorToInstantiation.put(reactor, instantiation)
        this.reactorToDecl.put(reactor, decl)
        
        if (!container.isMain && !container.isFederated) {
            this.addEdge(container, reactor)
        } else {
            this.addNode(reactor)
        }
        for (inst : reactor.instantiations) {
            inst.collectDeclarations
        }
    }
}