/*************
 * Copyright (c) 2020, The University of California at Berkeley.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.scoping;

import static java.util.Collections.emptyList;
import static org.lflang.ast.ASTUtils.*;

import com.google.inject.Inject;
import java.util.ArrayList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.xtext.naming.SimpleNameProvider;
import org.eclipse.xtext.scoping.IScope;
import org.eclipse.xtext.scoping.Scopes;
import org.eclipse.xtext.scoping.impl.SelectableBasedScope;
import org.lflang.lf.Assignment;
import org.lflang.lf.Connection;
import org.lflang.lf.Deadline;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfPackage;
import org.lflang.lf.Mode;
import org.lflang.lf.Model;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.lf.Watchdog;

/**
 * This class enforces custom rules. In particular, it resolves references to parameters, ports,
 * actions, and timers. Ports can be referenced across at most one level of hierarchy. Parameters,
 * actions, and timers can be referenced locally, within the reactor.
 *
 * @author Marten Lohstroh
 * @see <a href="https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#scoping"></a>
 */
public class LFScopeProviderImpl extends AbstractLFScopeProvider {

  @Inject private SimpleNameProvider nameProvider;

  @Inject private LFGlobalScopeProvider scopeProvider;

  /** Enumerate of the kinds of references. */
  enum RefType {
    NULL,
    TRIGGER,
    SOURCE,
    EFFECT,
    WATCHDOG,
    DEADLINE,
    CLEFT,
    CRIGHT
  }

  /**
   * Depending on the provided context, construct the appropriate scope for the given reference.
   *
   * @param context The AST node in which a to-be-resolved reference occurs.
   * @param reference The reference to resolve.
   */
  @Override
  public IScope getScope(EObject context, EReference reference) {
    if (context instanceof VarRef) {
      return getScopeForVarRef((VarRef) context, reference);
    } else if (context instanceof Assignment) {
      return getScopeForAssignment((Assignment) context, reference);
    } else if (context instanceof Instantiation) {
      return getScopeForReactorDecl(context, reference);
    } else if (context instanceof Reactor) {
      return getScopeForReactorDecl(context, reference);
    } else if (context instanceof ImportedReactor) {
      return getScopeForImportedReactor((ImportedReactor) context, reference);
    }
    return super.getScope(context, reference);
  }

  /**
   * Filter out candidates that do not originate from the file listed in this particular import
   * statement.
   */
  protected IScope getScopeForImportedReactor(ImportedReactor context, EReference reference) {
    String importURI = ((Import) context.eContainer()).getImportURI();
    var importedURI =
        scopeProvider.resolve(importURI == null ? "" : importURI, context.eResource());
    if (importedURI != null) {
      var uniqueImportURIs = scopeProvider.getImportedUris(context.eResource());
      var descriptions =
          scopeProvider.getResourceDescriptions(context.eResource(), uniqueImportURIs);
      var description = descriptions.getResourceDescription(importedURI);
      return SelectableBasedScope.createScope(
          IScope.NULLSCOPE, description, null, reference.getEReferenceType(), false);
    }
    return Scopes.scopeFor(emptyList());
  }

  /**
   * @param obj Instantiation or Reactor that has a ReactorDecl to resolve.
   * @param reference The reference to link to a ReactorDecl node.
   */
  protected IScope getScopeForReactorDecl(EObject obj, EReference reference) {

    // Find the local Model
    Model model = null;
    EObject container = obj;
    while (model == null && container != null) {
      container = container.eContainer();
      if (container instanceof Model) {
        model = (Model) container;
      }
    }
    if (model == null) {
      return Scopes.scopeFor(emptyList());
    }

    // Collect eligible candidates, all of which are local (i.e., not in other files).
    var locals = new ArrayList<ReactorDecl>(model.getReactors());

    // Either point to the import statement (if it is renamed)
    // or directly to the reactor definition.
    for (Import it : model.getImports()) {
      for (ImportedReactor ir : it.getReactorClasses()) {
        if (ir.getName() != null) {
          locals.add(ir);
        } else if (ir.getReactorClass() != null) {
          locals.add(ir.getReactorClass());
        }
      }
    }
    return Scopes.scopeFor(locals);
  }

  protected IScope getScopeForAssignment(Assignment assignment, EReference reference) {

    if (reference == LfPackage.Literals.ASSIGNMENT__LHS) {
      var defn = toDefinition(((Instantiation) assignment.eContainer()).getReactorClass());
      if (defn != null) {
        return Scopes.scopeFor(allParameters(defn));
      }
    }
    if (reference == LfPackage.Literals.ASSIGNMENT__RHS) {
      return Scopes.scopeFor(((Reactor) assignment.eContainer().eContainer()).getParameters());
    }
    return Scopes.scopeFor(emptyList());
  }

  protected IScope getScopeForVarRef(VarRef variable, EReference reference) {
    if (reference == LfPackage.Literals.VAR_REF__VARIABLE) {
      // Resolve hierarchical reference
      Reactor reactor;
      Mode mode = null;
      if (variable.eContainer().eContainer() instanceof Reactor) {
        reactor = (Reactor) variable.eContainer().eContainer();
      } else if (variable.eContainer().eContainer() instanceof Mode) {
        mode = (Mode) variable.eContainer().eContainer();
        reactor = (Reactor) variable.eContainer().eContainer().eContainer();
      } else {
        return Scopes.scopeFor(emptyList());
      }

      RefType type = getRefType(variable);

      if (variable.getContainer() != null) { // Resolve hierarchical port reference
        var instanceName = nameProvider.getFullyQualifiedName(variable.getContainer());
        var instances = new ArrayList<Instantiation>(reactor.getInstantiations());
        if (mode != null) {
          instances.addAll(mode.getInstantiations());
        }

        if (instanceName != null) {
          for (var instance : instances) {
            var defn = toDefinition(instance.getReactorClass());
            if (defn != null && instance.getName().equals(instanceName.toString())) {
              switch (type) {
                case TRIGGER:
                case SOURCE:
                case CLEFT:
                  return Scopes.scopeFor(allOutputs(defn));
                case EFFECT:
                case DEADLINE:
                case CRIGHT:
                  return Scopes.scopeFor(allInputs(defn));
              }
            }
          }
        }
        return Scopes.scopeFor(emptyList());
      } else {
        // Resolve local reference
        switch (type) {
          case TRIGGER:
            {
              var candidates = new ArrayList<EObject>();
              if (mode != null) {
                candidates.addAll(mode.getActions());
                candidates.addAll(mode.getTimers());
              }
              candidates.addAll(allInputs(reactor));
              candidates.addAll(allActions(reactor));
              candidates.addAll(allTimers(reactor));
              candidates.addAll(allWatchdogs(reactor));
              return Scopes.scopeFor(candidates);
            }
          case SOURCE:
            return super.getScope(variable, reference);
          case EFFECT:
            {
              var candidates = new ArrayList<EObject>();
              if (mode != null) {
                candidates.addAll(mode.getActions());
                candidates.addAll(reactor.getModes());
              }
              candidates.addAll(allOutputs(reactor));
              candidates.addAll(allActions(reactor));
              candidates.addAll(allWatchdogs(reactor));
              return Scopes.scopeFor(candidates);
            }
          case WATCHDOG:
            return Scopes.scopeFor(allWatchdogs(reactor));
          case DEADLINE:
          case CLEFT:
            return Scopes.scopeFor(allInputs(reactor));
          case CRIGHT:
            return Scopes.scopeFor(allOutputs(reactor));
          default:
            return Scopes.scopeFor(emptyList());
        }
      }
    } else { // Resolve instance
      return super.getScope(variable, reference);
    }
  }

  private RefType getRefType(VarRef variable) {
    if (variable.eContainer() instanceof Deadline) {
      return RefType.DEADLINE;
    } else if (variable.eContainer() instanceof Reaction) {
      var reaction = (Reaction) variable.eContainer();
      if (reaction.getTriggers().contains(variable)) {
        return RefType.TRIGGER;
      } else if (reaction.getSources().contains(variable)) {
        return RefType.SOURCE;
      } else if (reaction.getEffects().contains(variable)) {
        return RefType.EFFECT;
      }
    } else if (variable.eContainer() instanceof Connection) {
      var conn = (Connection) variable.eContainer();
      if (conn.getLeftPorts().contains(variable)) {
        return RefType.CLEFT;
      } else if (conn.getRightPorts().contains(variable)) {
        return RefType.CRIGHT;
      }
    } else if (variable.eContainer() instanceof Watchdog) {
      var watchdog = (Watchdog) variable.eContainer();
      if (watchdog.getEffects().contains(variable)) {
        return RefType.EFFECT;
      }
    }
    return RefType.NULL;
  }
}
