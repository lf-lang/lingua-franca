package org.lflang;

import org.eclipse.emf.ecore.EObject;

public interface ErrorReporter {
    
    /**
     * Report an error.
     * @param message The error message.
     * @return a string that describes the error.
     */
    String reportError(String message);

    /**
     * Report a warning.
     * @param message The warning message.
     * @return a string that describes the error.
     */
    String reportWarning(String message);


    /** 
     * Report an error on the specified parse tree object.
     * @param object The parse tree object.
     * @param message The error message.
     * @return a string that describes the error.
     */
    String reportError(EObject object, String message);

    /** 
     * Report a warning on the specified parse tree object.
     * @param object The parse tree object.
     * @param message The error message.
     * @return a string that describes the error.
     */
    String reportWarning(EObject object, String message);
    
}
