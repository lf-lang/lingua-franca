/*
 * KIELER - Kiel Integrated Environment for Layout Eclipse RichClient
 *
 * http://rtsys.informatik.uni-kiel.de/kieler
 * 
 * Copyright 2021 by
 * + Kiel University
 *   + Department of Computer Science
 *     + Real-Time and Embedded Systems Group
 * 
 * This code is provided under the terms of the Eclipse Public License (EPL).
 */
package org.lflang.diagram.synthesis.util

import org.lflang.ErrorReporter
import org.eclipse.emf.ecore.EObject

/**
 * @author als
 */
class SynthesisErrorReporter implements ErrorReporter {
    
    override reportError(String message) {
    }
    
    override reportError(EObject object, String message) {
    }
    
    override reportWarning(String message) {
    }
    
    override reportWarning(EObject object, String message) {
    }
    
}