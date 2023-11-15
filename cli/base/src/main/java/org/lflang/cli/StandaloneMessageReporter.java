/*************
 * Copyright (c) 2019, The University of California at Berkeley. Copyright (c)
 * 2019, TU Dresden
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

package org.lflang.cli;

import com.google.inject.Inject;
import java.nio.file.Path;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.xtext.diagnostics.Severity;
import org.lflang.MessageReporterBase;
import org.lflang.generator.Range;

/**
 * An error reporter that forwards all messages to an {@link IssueCollector}. They'll be sorted out
 * later.
 */
public class StandaloneMessageReporter extends MessageReporterBase {

  @Inject private StandaloneIssueAcceptor issueAcceptor;

  static Severity convertSeverity(DiagnosticSeverity severity) {
    return switch (severity) {
      case Error -> Severity.ERROR;
      case Warning -> Severity.WARNING;
      case Information, Hint -> Severity.INFO;
    };
  }

  @Override
  protected void reportOnNode(
      EObject node, EStructuralFeature feature, DiagnosticSeverity severity, String message) {
    issueAcceptor.accept(convertSeverity(severity), message, node, feature, 0, null);
  }

  @Override
  protected void report(Path path, Range range, DiagnosticSeverity severity, String message) {
    LfIssue issue = new LfIssue(message, convertSeverity(severity), path, range);
    issueAcceptor.accept(issue);
  }

  @Override
  protected void reportWithoutPosition(DiagnosticSeverity severity, String message) {
    LfIssue issue = new LfIssue(message, convertSeverity(severity), null, null);
    issueAcceptor.accept(issue);
  }

  @Override
  public boolean getErrorsOccurred() {
    return issueAcceptor.getErrorsOccurred();
  }
}
