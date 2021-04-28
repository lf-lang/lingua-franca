/*
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.lflang.scoping

import com.google.inject.Inject
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.EReference
import org.eclipse.xtext.naming.SimpleNameProvider
import org.eclipse.xtext.scoping.IScope
import org.eclipse.xtext.scoping.Scopes
import org.eclipse.xtext.scoping.impl.SelectableBasedScope
import org.eclipse.xtext.xbase.lib.CollectionLiterals
import org.lflang.*
import org.lflang.lf.*
import java.util.Collections.emptyList

/**
 * This class enforces custom rules. In particular, it resolves references to
 * parameters, ports, actions, and timers. Ports can be referenced across at
 * most one level of hierarchy. Parameters, actions, and timers can be
 * referenced locally, within the reactor.
 *
 * @see "https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html.scoping on how and when to use it."
 *
 * @author Marten Lohstroh
 */
class LFScopeProvider : AbstractLFScopeProvider() {

    @Inject
    lateinit var nameProvider: SimpleNameProvider

    @Inject
    lateinit var scopeProvider: LFGlobalScopeProvider


    /**
     * Enumerate of the kinds of references.
     */
    private enum class RefType {
        NULL, TRIGGER, SOURCE, EFFECT, DEADLINE, CLEFT, CRIGHT
    }

    /**
     * Depending on the provided context, construct the appropriate scope
     * for the given reference.
     * @param context The AST node in which a to-be-resolved reference occurs.
     * @param reference The reference to resolve.
     */
    override fun getScope(context: EObject, reference: EReference): IScope {
        return when (context) {
            is VarRef          -> getScopeForVarRef(context, reference)
            is Assignment      -> getScopeForAssignment(context, reference)
            is Instantiation   -> getScopeForReactorDecl(context, reference)
            is Reactor         -> getScopeForReactorDecl(context, reference)
            is ImportedReactor -> getScopeForImportedReactor(context, reference)
            else               -> super.getScope(context, reference)
        }
    }

    /**
     * Filter out candidates that do not originate from the file listed in
     * this particular import statement.
     */
    private fun getScopeForImportedReactor(context: ImportedReactor, reference: EReference): IScope {
        val importURI = (context.eContainer() as Import).importURI ?: ""
        val importedURI = scopeProvider.resolve(importURI, context.eResource())
        if (importedURI != null) {
            val uniqueImportURIs: Set<URI> = scopeProvider.getImportedUris(context.eResource())
            val uri = uniqueImportURIs.first { it == importedURI }
            val descriptions = scopeProvider.getResourceDescriptions(context.eResource(), uniqueImportURIs)
            val description = descriptions.getResourceDescription(uri)
            return SelectableBasedScope.createScope(IScope.NULLSCOPE, description, null, reference.eReferenceType, false)
        }
        return Scopes.scopeFor(CollectionLiterals.newLinkedList())
    }

    /**
     * @param obj Instantiation or Reactor that has a ReactorDecl to resolve.
     * @param reference The reference to link to a ReactorDecl node.
     */
    private fun getScopeForReactorDecl(obj: EObject, reference: EReference?): IScope {
        val locals = mutableListOf<EObject>()

        // Find the local Model
        val model = obj.eContainer() as? Model
                ?: obj.eContainer().eContainer() as? Model
                ?: return Scopes.scopeFor(emptyList())

        // Collect eligible candidates, all of which are local (i.e., not in other files).
        model.reactors?.forEach { locals.add(it) }
        model.imports?.forEach { import ->
            // Either point to the import statement (if it is renamed)
            // or directly to the reactor definition.
            import.reactorClasses?.forEach {
                if (it.name != null) locals.add(it)
                else if (it.reactorClass != null) locals.add(it.reactorClass)
            }
        }
        return Scopes.scopeFor(locals)
    }

    private fun getScopeForAssignment(assignment: Assignment, reference: EReference?): IScope {

        if (reference == LfPackage.Literals.ASSIGNMENT__LHS) {
            val defn = ((assignment.eContainer() as Instantiation).reactorClass)?.toDefinition()
            if (defn != null) {
                return Scopes.scopeFor(defn.allParameters)
            }
        }
        if (reference == LfPackage.Literals.ASSIGNMENT__RHS) {
            return Scopes.scopeFor((assignment.eContainer().eContainer() as Reactor).parameters)
        }
        return Scopes.scopeFor(emptyList())

    }

    private fun getScopeForVarRef(variable: VarRef, reference: EReference?): IScope {
        if (reference != LfPackage.Literals.VAR_REF__VARIABLE) {
            return super.getScope(variable, reference)
        }

        val reactor: Reactor = variable.eContainer().eContainer() as? Reactor
                ?: return Scopes.scopeFor(emptyList())

        val type = when (val eContainer = variable.eContainer()) {
            is Deadline   -> RefType.DEADLINE
            is Reaction   -> when (variable) {
                in eContainer.triggers -> RefType.TRIGGER
                in eContainer.sources  -> RefType.SOURCE
                in eContainer.effects  -> RefType.EFFECT
                else                   -> RefType.NULL
            }
            is Connection -> when (variable) {
                in eContainer.leftPorts  -> RefType.CLEFT
                in eContainer.rightPorts -> RefType.CRIGHT
                else                     -> RefType.NULL
            }
            else          -> RefType.NULL
        }

        if (variable.container != null) { // Resolve hierarchical port reference
            val instanceName = nameProvider.getFullyQualifiedName(variable.container)
            val instances = reactor.instantiations

            for (instance in instances) {
                val defn = instance.reactorClass?.toDefinition()
                if (defn != null && instanceName != null && instance.name == instanceName.toString()) {
                    return when (type) {
                        RefType.TRIGGER, RefType.SOURCE, RefType.CLEFT   -> Scopes.scopeFor(defn.allOutputs)
                        RefType.EFFECT, RefType.DEADLINE, RefType.CRIGHT -> Scopes.scopeFor(defn.allInputs)
                        else                                             -> Scopes.scopeFor(emptyList())
                    }
                }
            }
        }
        if (type == RefType.SOURCE)
            return super.getScope(variable, reference)

        val candidates: List<EObject> = when (type) {
            RefType.TRIGGER  -> mutableListOf<EObject>().apply {
                this += reactor.allInputs
                this += reactor.allActions
                this += reactor.allTimers
            }
            RefType.EFFECT   -> reactor.allOutputs + reactor.allActions
            RefType.DEADLINE -> reactor.allInputs
            RefType.CLEFT    -> reactor.allInputs
            RefType.CRIGHT   -> reactor.allOutputs
            else             -> emptyList()
        }

        return Scopes.scopeFor(candidates)
    }
}
