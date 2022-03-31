/*
 * KIELER - Kiel Integrated Environment for Layout Eclipse RichClient
 *
 * http://rtsys.informatik.uni-kiel.de/kieler
 * 
 * Copyright 2022 by
 * + Kiel University
 *   + Department of Computer Science
 *     + Real-Time and Embedded Systems Group
 * 
 * This code is provided under the terms of the Eclipse Public License (EPL).
 */
package org.lflang.ui.diagram;

import java.util.HashMap;
import java.util.Iterator;

import org.eclipse.elk.core.util.Pair;
import org.eclipse.emf.ecore.EObject;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;

import com.google.common.base.Function;
import com.google.common.collect.Iterators;

import de.cau.cs.kieler.klighd.KlighdTreeSelection;

/**
 * @author als
 */
public class LinguaFrancaAdjustedKlighdTreeSelection extends KlighdTreeSelection {
    
    /** The LF model */
    private final Model model;
    /** Cached map of imports */
    private final HashMap<Reactor, ImportedReactor> importedReactors = new HashMap<Reactor, ImportedReactor>();
    
    /**
     * Create LF adjusted wrapper around KlighdTreeSelection.
     * 
     * @param source copy data from original KlighdTreeSelection
     */
    LinguaFrancaAdjustedKlighdTreeSelection(KlighdTreeSelection source, Model lfModel) {
        super(source.getViewContext(), source.getPaths());
        
        model = lfModel;
        // Create import map
        for (var imp : lfModel.getImports()) {
            for (var impReactor : imp.getReactorClasses()) {
                importedReactors.put(impReactor.getReactorClass(), impReactor);
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Iterator<Object> sourceElementIterator() {
        return Iterators.transform(iterator(),
                new Function<EObject, Object>() {
                    public Object apply(final EObject diagramElement) {
                        var source = getViewContext().getSourceElement(diagramElement);
                        // Adjust for imported elements
                        if (!importedReactors.isEmpty() && source instanceof EObject) {
                            var sourceEObj = (EObject) source;
                            if (sourceEObj.eResource() != model.eResource()) { // If this source element is imported
                                source = findImport(sourceEObj, diagramElement);
                            }
                        }
                        return source;
                    }
                });
    }
    
    /**
     * {@inheritDoc}
     */
    @Override
    public Iterator<Pair<EObject, Object>> sourceViewPairIterator() {
        return Iterators.transform(iterator(),
                new Function<EObject, Pair<EObject, Object>>() {
                    public Pair<EObject, Object> apply(final EObject diagramElement) {
                        var source = getViewContext().getSourceElement(diagramElement);
                        // Adjust for imported elements
                        if (!importedReactors.isEmpty() && source instanceof EObject) {
                            var sourceEObj = (EObject) source;
                            if (sourceEObj.eResource() != model.eResource()) { // If this source element is imported
                                source = findImport(sourceEObj, diagramElement);
                            }
                        }
                        return Pair.of(diagramElement, source);
                    }
                });
    }

    /**
     * Find the reactor import for the given model element.
     * 
     * @param sourceEObj the original associate
     * @param diagramElement the diagram element
     * @return the associated reactor import or the original associate
     */
    private Object findImport(EObject sourceEObj, EObject diagramElement) {
        var parent = diagramElement;
        do {
            var parentSource = getViewContext().getSourceElement(parent);
            if (importedReactors.containsKey(parentSource)) {
                return importedReactors.get(parentSource); // associate with import
            }
            parent = parent.eContainer();
        } while (parent != null);
        return sourceEObj; // keep old association
    }
    
}
