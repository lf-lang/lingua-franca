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
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;
import org.lflang.MessageReporterBase;
import org.lflang.generator.Range;

/**
 * This class translates messages reported via the ErrorReporrter interface to the interface of a
 * given ValidationMessageAcceptor.
 *
 * <p>Effectively this allows to report errors via the ErrorReporter interface during validator
 * checks, while having the validator still track all the reported warnings and messages. This is
 * required for some functionality, like the construction of an instance graph in
 * LFValidator.checkModel(). Since the instance graph is also used in other components, it does not
 * report directly to the validator, but uses our custom ErrorReporter interface that we use during
 * code generation. This class bridges the gap between the ErrorReporter interface and the messages
 * that the validator expects.
 *
 * @author Christian Menard
 */
public class ValidatorMessageReporter extends MessageReporterBase {

  private final ValidationMessageAcceptor acceptor;
  private final BaseLFValidator.ValidatorStateAccess validatorState;

  public ValidatorMessageReporter(
      ValidationMessageAcceptor acceptor, BaseLFValidator.ValidatorStateAccess stateAccess) {
    this.acceptor = acceptor;
    this.validatorState = stateAccess;
  }

  @Override
  protected void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    reportOnNode(validatorState.getCurrentObject(), severity, message);
  }

  /**
   * Report the given message as an error on the current object.
   *
   * <p>Unfortunately, there is no way to provide a path and a line number to the
   * ValidationMessageAcceptor as messages can only be reported directly as EObjects. While it is
   * not an ideal solution, this method composes a messages indicating the location of the error and
   * reports this on the object currently under validation. This way, the error message is not lost,
   * but it is not necessarily reported precisely at the location of the actual error.
   */
  @Override
  protected void report(Path path, Range range, DiagnosticSeverity severity, String message) {
    String fullMessage =
        message
            + " (Reported from "
            + path
            + " on line "
            + range.getStartInclusive().getOneBasedLine()
            + ")";
    reportOnNode(validatorState.getCurrentObject(), severity, fullMessage);
  }

  @Override
  protected void reportOnNode(
      EObject node, EStructuralFeature feature, DiagnosticSeverity severity, String message) {
    switch (severity) {
      case Error ->
          acceptor.acceptError(
              message, node, feature, ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
      case Warning ->
          acceptor.acceptWarning(
              message, node, feature, ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
      case Information, Hint ->
          acceptor.acceptInfo(
              message, node, feature, ValidationMessageAcceptor.INSIGNIFICANT_INDEX, null);
    }
  }
}
