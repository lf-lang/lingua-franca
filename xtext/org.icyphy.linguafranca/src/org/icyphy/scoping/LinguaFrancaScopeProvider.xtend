/* Scope provider for Lingua Franca. */

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

package org.icyphy.scoping

import com.google.inject.Inject
import java.util.ArrayList
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.EReference
import org.eclipse.xtext.naming.SimpleNameProvider
import org.eclipse.xtext.scoping.Scopes
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Deadline
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaPackage
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.VarRef

import static extension org.icyphy.ASTUtils.*
import org.icyphy.linguaFranca.ImportedReactor
import org.eclipse.xtext.resource.IResourceDescription
import org.eclipse.emf.common.util.URI
import org.eclipse.xtext.scoping.impl.FilteringScope
import org.eclipse.xtext.scoping.impl.ImportUriResolver

/**
 * This class enforces custom rules. In particular, it resolves references to 
 * parameters, ports, actions, and timers. Ports can be referenced across at
 * most one level of hierarchy. Parameters, actions, and timers can be 
 * referenced locally, within the reactor.
 * 
 * @see https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#scoping
 * on how and when to use it.
 * @author Marten Lohstroh
 */
class LinguaFrancaScopeProvider extends AbstractLinguaFrancaScopeProvider {

    @Inject
    SimpleNameProvider nameProvider
    
//    @Inject
//    IResourceDescription.Manager descriptionManager;
//
//    @Inject
//    ImportUriResolver uriResolver
    
    /**
     * 
     */
    protected enum RefType {
        NULL,
        TRIGGER,
        SOURCE,
        EFFECT,
        DEADLINE,
        CLEFT,
        CRIGHT
    }
    
    /**
     * 
     */
    override getScope(EObject context, EReference reference) {
        switch (context) {
            VarRef: return getScopeForVarRef(context, reference)
            Assignment: return getScopeForAssignment(context, reference)
            Instantiation: return getScopeForReactorDecl(context, reference)
            Reactor: return getScopeForReactorDecl(context, reference)
            ImportedReactor: return getScopeForImportedReactor(context, reference)
        }
        return super.getScope(context, reference);
    }
    
    /**
     * Filter out candidates that do not originate from the file listed in
     * this particular import statement.
     */
    protected def getScopeForImportedReactor(ImportedReactor context,
        EReference reference) {

        val importedURI = URI.createURI((context.eContainer as Import).importURI ?: "").resolve(context.eResource.URI)

        return new FilteringScope(super.getScope(context, reference), [ iod |
            iod.EObjectURI.toString.split('#').get(0).equals(importedURI.toString)
        ])
    }
    
    /**
     * 
     */
    protected def getScopeForReactorDecl(EObject obj, EReference reference) {
        var Model model
        val locals = newLinkedList
        
        // Find the local Model
        if (obj.eContainer instanceof Model) {
            model = obj.eContainer as Model
        } else if (obj.eContainer.eContainer instanceof Model) {
            model = obj.eContainer.eContainer as Model
        } else {
             // Empty list
        }
        
        // Collect eligible candidates, all of which are local (i.e., not in other files).
        model.reactors?.forEach[locals.add(it)]
        model.imports?.forEach [
            it.reactorClasses?.forEach [
                (it.name !== null) ? locals.add(it);
                (it.reactorClass !== null) ? locals.add(it.reactorClass)
            ]
        ]
        return Scopes.scopeFor(locals)
    }
        
//    def getScopeForReactor(Reactor reactor, EReference reference) {
//        if (reference == LinguaFrancaPackage.IMPORTED_REACTOR__REACTOR_CLASS) {
//            // FIXME: find in resource set a model and extract its reactors
//            return super.getScope(reactor, reference)
//        }
//        return super.getScope(reactor, reference)
//    }
    
    
//    protected def getScopeForReactorDecl(ReactorDecl decl, EReference reference) {
//        
////        val candidates = newLinkedList
//        
//        if (decl instanceof Reactor) {
//            return super.getScope(decl, reference)
//        }
//        val locals = newLinkedList
////        
//        val model = decl.eContainer.eContainer as Model
////        
//        model.reactors.forEach[locals.add(it)]
//        model.imports.forEach[it.reactorClasses.forEach[locals.add(it); locals.add(it.reactorClass)]]
////        super.getScope(decl, reference).getElements(nameProvider.
////                    getFullyQualifiedName(decl)).filter[found | locals.exists[it.name == found.name]].forEach[candidates.add(it.EObjectOrProxy)]
////        val x = super.getScope(decl, reference);
////        return x
//        return Scopes.scopeFor(locals)
//        
////        if (reference == LinguaFrancaPackage.Literals.REACTOR__SUPER_CLASSES) {
////            
////        }
////        
////        Scopes.scopeFor(candidates)
//    }

    protected def getScopeForAssignment(Assignment assignment,
        EReference reference) {
        
        if (reference == LinguaFrancaPackage.Literals.ASSIGNMENT__LHS) {
            val defn = (assignment.eContainer as Instantiation).reactorClass.toDefinition
            if (defn !== null) {
                return Scopes.scopeFor(defn.parameters)
            }
            
        }
        if (reference == LinguaFrancaPackage.Literals.ASSIGNMENT__RHS) {
            return Scopes.scopeFor(
                (assignment.eContainer.eContainer as Reactor).parameters)
        }
        return Scopes.scopeFor(newLinkedList)
    }

    protected def getScopeForVarRef(VarRef variable, EReference reference) {
        if (reference == LinguaFrancaPackage.Literals.VAR_REF__VARIABLE) {
            // Resolve hierarchical reference
            val candidates = new ArrayList<EObject>()
            var type = RefType.NULL
            var Reactor reactor = null

            if (variable.eContainer.eContainer instanceof Reactor) {
                reactor = variable.eContainer.eContainer as Reactor
            } else {
                return Scopes.scopeFor(newLinkedList)
            }

            if (variable.eContainer instanceof Deadline) {
                type = RefType.DEADLINE
            } else if (variable.eContainer instanceof Reaction) {
                val reaction = variable.eContainer as Reaction
                if (reaction.triggers.contains(variable)) {
                    type = RefType.TRIGGER
                } else if (reaction.sources.contains(variable)) {
                    type = RefType.SOURCE
                } else if (reaction.effects.contains(variable)) {
                    type = RefType.EFFECT
                }
            } else if (variable.eContainer instanceof Connection) {
                val conn = variable.eContainer as Connection
                if (conn.leftPort === variable) {
                    type = RefType.CLEFT
                } else if (conn.rightPort === variable) {
                    type = RefType.CRIGHT
                }
            }

            if (variable.container !== null) { // Resolve hierarchical port reference
                val instanceName = nameProvider.
                    getFullyQualifiedName(variable.container)
                val instances = reactor.instantiations
                
                for (instance : instances) {
                    val defn = instance.reactorClass.toDefinition
                    if (defn !== null && instanceName !== null &&
                        instance.name.equals(instanceName.toString)) {
                        if (type === RefType.TRIGGER ||
                            type === RefType.SOURCE || type === RefType.CLEFT) {
                            return Scopes.scopeFor(
                                defn.outputs)
                        } else if (type === RefType.EFFECT ||
                            type === RefType.DEADLINE ||
                            type === RefType.CRIGHT) {
                            return Scopes.scopeFor(defn.inputs)
                        }
                    }
                }
                return Scopes.scopeFor(candidates) // Empty list
            } else { // Resolve local reference
                switch (type) {
                    case RefType.TRIGGER: {
                        candidates.addAll(reactor.inputs)
                        candidates.addAll(reactor.actions)
                        candidates.addAll(reactor.timers)
                    }
                    case RefType.SOURCE:
                        return super.getScope(variable, reference)
                    case RefType.EFFECT: {
                        candidates.addAll(reactor.outputs)
                        candidates.addAll(reactor.actions)
                    }
                    case RefType.DEADLINE:
                        return Scopes.scopeFor(reactor.inputs)
                    case RefType.CLEFT:
                        return Scopes.scopeFor(reactor.inputs)
                    case RefType.CRIGHT:
                        return Scopes.scopeFor(reactor.outputs)
                    default: {
                    }
                }
                return Scopes.scopeFor(candidates)
            }
        } else { // Resolve instance
            return super.getScope(variable, reference)
        }
    }
}
