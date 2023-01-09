/*************
 * Copyright (c) 2021, TU Dresden.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.validation;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;

import org.lflang.ErrorReporter;

/**
 * This class translates messages reported via the ErrorReporrter interface to
 * the interface of a given ValidationMessageAcceptor.
 * 
 * Effectively this allows to report errors via the ErrorReporter interface
 * during validator checks, while having the validator still track all the
 * reported warnings and messages. This is required for some functionality, like
 * the construction of an instance graph in LFValidator.checkModel(). Since the
 * instance graph is also used in other components, it does not report directly
 * to the validator, but uses our custom ErrorReporter interface that we use
 * during code generation. This class bridges the gap between the ErrorReporter
 * interface and the messages that the validator expects.
 * 
 * @author Christian Menard
 */
public class ValidatorErrorReporter implements ErrorReporter {

    private ValidationMessageAcceptor acceptor;
    private BaseLFValidator.ValidatorStateAccess validatorState;
    private boolean errorsOccurred = false;

    public ValidatorErrorReporter(ValidationMessageAcceptor acceptor,
            BaseLFValidator.ValidatorStateAccess stateAccess) {
        this.acceptor = acceptor;
        this.validatorState = stateAccess;
    }

    /**
     * Report the given message as an error on the object currently under
     * validation.
     */
    @Override
    public String reportError(String message) {
        errorsOccurred = true;
        acceptor.acceptError(message, validatorState.getCurrentObject(), null,
                ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return message;
    }

    /**
     * Report the given message as an error on the given object.
     */
    @Override
    public String reportError(EObject object, String message) {
        errorsOccurred = true;
        acceptor.acceptError(message, object, null,
                ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return message;
    }

    /**
     * Report the given message as an error on the current object.
     * 
     * Unfortunately, there is no way to provide a path and a line number to the
     * ValidationMessageAcceptor as messages can only be reported directly as
     * EObjects. While it is not an ideal solution, this method composes a
     * messages indicating the location of the error and reports this on the
     * object currently under validation. This way, the error message is not
     * lost, but it is not necessarily reported precisely at the location of the
     * actual error.
     */
    @Override
    public String reportError(Path file, Integer line, String message) {
        errorsOccurred = true;
        String fullMessage = message + " (Reported from " + file.toString() + " on line "
            + line.toString() + ")";
        acceptor.acceptError(fullMessage, validatorState.getCurrentObject(),
                null, ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return fullMessage;
    }

    /**
     * Report the given message as a waring on the object currently under
     * validation.
     */
    @Override
    public String reportWarning(String message) {
        acceptor.acceptWarning(message, validatorState.getCurrentObject(), null,
                ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return message;
    }

    @Override
    public String reportInfo(String message) {
        acceptor.acceptInfo(message, validatorState.getCurrentObject(), null,
                               ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return message;
    }

    /**
     * Report the given message as a warning on the given object.
     */
    @Override
    public String reportWarning(EObject object, String message) {
        acceptor.acceptWarning(message, object, null,
                ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return message;
    }

    @Override
    public String reportInfo(EObject object, String message) {
        acceptor.acceptInfo(message, object, null,
                               ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return message;
    }


    /**
     * Report the given message as an warning on the current object.
     * 
     * Unfortunately, there is no way to provide a path and a line number to the
     * ValidationMessageAcceptor as messages can only be reported directly as
     * EObjects. While it is not an ideal solution, this method composes a
     * messages indicating the location of the warning and reports this on the
     * object currently under validation. This way, the warning message is not
     * lost, but it is not necessarily reported precisely at the location of the
     * actual warning.
     */
    @Override
    public String reportWarning(Path file, Integer line, String message) {
        String fullMessage = message + " (Reported from " + file.toString() + " on line "
            + line.toString() + ")";
        acceptor.acceptWarning(fullMessage, validatorState.getCurrentObject(),
                null, ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return fullMessage;
    }

    @Override
    public String reportInfo(Path file, Integer line, String message) {
        String fullMessage = message + " (Reported from " + file.toString() + " on line "
            + line.toString() + ")";
        acceptor.acceptInfo(fullMessage, validatorState.getCurrentObject(),
                               null, ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
        return fullMessage;
    }

    /**
     * Check if errors where reported.
     *
     * @return true if errors where reported
     */
    @Override
    public boolean getErrorsOccurred() {
        return errorsOccurred;
    }
}
