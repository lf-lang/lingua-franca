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
import static org.lflang.ast.ASTUtils.allActions;
import static org.lflang.ast.ASTUtils.allInputs;
import static org.lflang.ast.ASTUtils.allInstantiations;
import static org.lflang.ast.ASTUtils.allOutputs;
import static org.lflang.ast.ASTUtils.allParameters;
import static org.lflang.ast.ASTUtils.allTimers;
import static org.lflang.ast.ASTUtils.allWatchdogs;
import static org.lflang.ast.ASTUtils.toDefinition;

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
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;
import org.lflang.lf.Watchdog;
import org.lflang.lf.WidthTerm;
import org.lflang.util.ImportUtil;

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
    } else if (context instanceof WidthTerm) {
      return getScopeForWidthTerm((WidthTerm) context, reference);
    } else if (context instanceof ParameterReference) {
      return getScopeForParameterReference((ParameterReference) context, reference);
    }
    return super.getScope(context, reference);
  }

  /**
   * Filter out candidates that do not originate from the file listed in this particular import
   * statement.
   */
  protected IScope getScopeForImportedReactor(ImportedReactor context, EReference reference) {
    String importURI =
        ((Import) context.eContainer()).getImportURI() != null
            ? ((Import) context.eContainer()).getImportURI()
            : ImportUtil.buildPackageURI(
                ((Import) context.eContainer()).getImportPackage(), context.eResource());
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
    Model model = enclosingModel(obj);
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

  protected IScope getScopeForWidthTerm(WidthTerm term, EReference reference) {
    // Find the nearest containing reactor. A WidthTerm is within a WidthSpec,
    // which is within a Port or an Instantiation.  So the nearest possibility
    // is three levels up.
    EObject reactor = enclosingReactor(term);
    if (reactor == null) {
      return Scopes.scopeFor(emptyList());
    }
    return Scopes.scopeFor(allParameters((Reactor) reactor));
  }

  protected IScope getScopeForParameterReference(ParameterReference term, EReference reference) {
    EObject reactor = enclosingReactor(term);
    if (reactor == null) {
      return Scopes.scopeFor(emptyList());
    }
    return Scopes.scopeFor(allParameters((Reactor) reactor));
  }

  protected IScope getScopeForAssignment(Assignment assignment, EReference reference) {

    if (reference == LfPackage.Literals.ASSIGNMENT__LHS) {
      var defn = toDefinition(((Instantiation) assignment.eContainer()).getReactorClass());
      if (defn != null) {
        return Scopes.scopeFor(allParameters(defn));
      }
    }
    if (reference == LfPackage.Literals.ASSIGNMENT__RHS) {
      Reactor reactor = enclosingReactor(assignment);
      return Scopes.scopeFor(allParameters(reactor));
    }
    return Scopes.scopeFor(emptyList());
  }

  protected IScope getScopeForVarRef(VarRef variable, EReference reference) {
    Reactor reactor = enclosingReactor(variable);
    Mode mode = enclosingMode(variable);
    if (reactor == null) {
      return Scopes.scopeFor(emptyList());
    }

    if (reference == LfPackage.Literals.VAR_REF__VARIABLE) {
      // Resolve variable reference
      RefType type = getRefType(variable);

      if (variable.getContainer() != null) { // Resolve hierarchical port reference
        var instanceName = nameProvider.getFullyQualifiedName(variable.getContainer());
        var instances = new ArrayList<Instantiation>(allInstantiations(reactor));
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
    } else if (reference == LfPackage.Literals.VAR_REF__CONTAINER) {
      return Scopes.scopeFor(allInstantiations(reactor));
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

  /**
   * Return the nearest enclosing reactor or null if one is not found.
   *
   * @param term A term in the abstract syntax tree.
   */
  private Reactor enclosingReactor(EObject term) {
    EObject reactor = term.eContainer();
    while (reactor != null && !(reactor instanceof Reactor)) {
      reactor = reactor.eContainer();
    }
    return (Reactor) reactor;
  }

  /**
   * If the term is within a Mode before it is within a Reactor, then return that mode. Otherwise,
   * return null.
   *
   * @param term A term in the abstract syntax tree.
   */
  private Mode enclosingMode(EObject term) {
    EObject mode = term.eContainer();
    while (mode != null && !(mode instanceof Mode)) {
      if (mode instanceof Reactor) return null;
      mode = mode.eContainer();
    }
    return (Mode) mode;
  }

  /**
   * Return the enclosing Model or null if one is not found.
   *
   * @param term A term in the abstract syntax tree.
   */
  private Model enclosingModel(EObject term) {
    EObject model = term.eContainer();
    while (model != null && !(model instanceof Model)) {
      model = model.eContainer();
    }
    return (Model) model;
  }
}
