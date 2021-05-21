package org.lflang

import org.eclipse.emf.ecore.EObject

class DefaultErrorReporter implements ErrorReporter {
    
    public static val DEFAULT = new DefaultErrorReporter()
    
    override reportError(String message) {
        println(message)
    }
    
    override reportError(EObject object, String message) {
        println(message)
    }
    
    override reportWarning(String message) {
        println(message)
    }
    
    override reportWarning(EObject object, String message) {
        println(message)
    }
    
}