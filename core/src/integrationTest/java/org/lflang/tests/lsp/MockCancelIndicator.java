package org.lflang.tests.lsp;

import org.eclipse.xtext.util.CancelIndicator;

/**
 * Mock the CancelIndicator interface for testing.
 *
 * @author Erling Jellum
 */
public class MockCancelIndicator implements CancelIndicator {
  @Override
  public boolean isCanceled() {
    return false;
  }
}
