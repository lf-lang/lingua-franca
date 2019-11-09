package org.icyphy.scoping

import com.google.inject.Inject
import java.util.ArrayList
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.EReference
import org.eclipse.xtext.naming.SimpleNameProvider
import org.eclipse.xtext.scoping.Scopes
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Deadline
import org.icyphy.linguaFranca.Parameter
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Assignment
import org.icyphy.linguaFranca.Instantiation

/**
 * This class enforces custom rules. In particular, it resolves references to 
 * ports, actions, and timers. Ports can be referenced across at most one level
 * of hierarchy. Only local actions and timers can be referenced.
 * 
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#scoping
 * on how and when to use it.
 * @author Marten Lohstroh
 */
class LinguaFrancaScopeProvider extends AbstractLinguaFrancaScopeProvider {

	@Inject SimpleNameProvider nameProvider

	protected enum RefType {
		NULL,
		TRIGGER,
		SOURCE,
		EFFECT,
		DEADLINE,
		CLEFT,
		CRIGHT
	}

	override getScope(EObject context, EReference reference) {
		switch (context) {
			VarRef: return getScopeForVarRef(context, reference)
			Assignment: return getScopeForAssignment(context, reference)
		}
		return super.getScope(context, reference);
	}

	protected def getScopeForAssignment(Assignment assignment, EReference reference) {
		
		val candidates = new ArrayList<EObject>()
		if (reference.name === "lhs") {
			return Scopes.scopeFor((assignment.eContainer as Instantiation).reactorClass.parameters)	
		}
		if (reference.name === "rhs") {
			return Scopes.scopeFor((assignment.eContainer.eContainer as Reactor).parameters)
		}
		return Scopes.scopeFor(candidates)
	}

	protected def getScopeForVarRef(VarRef variable, EReference reference) {
		if (reference.name.equals("variable")) { // Resolve hierarchical reference
			val candidates = new ArrayList<EObject>()
			var type = RefType.NULL
			var reactor = null as Reactor
			
			if (variable.eContainer.eContainer instanceof Reactor) {
				reactor = variable.eContainer.eContainer as Reactor	
			} else {
				return Scopes.scopeFor([])
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
				val instanceName = nameProvider.getFullyQualifiedName(variable.container)
				val instances = reactor.instantiations
				for (instance : instances) {
					if (instanceName !== null && instance.name.equals(instanceName.toString)) {
						if (type === RefType.TRIGGER || type === RefType.SOURCE || type === RefType.CLEFT) {
							return Scopes.scopeFor(instance.reactorClass.outputs)
						} else if (type === RefType.EFFECT || type === RefType.DEADLINE || type === RefType.CRIGHT) {
							return Scopes.scopeFor(instance.reactorClass.inputs)
						}
					}
				}
				return Scopes.scopeFor(candidates) // empty set
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
