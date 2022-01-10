package org.lflang.tests.lsp;

import org.lflang.generator.IntegratedBuilder;

public class TestReportProgress implements IntegratedBuilder.ReportProgress {
    private int previousPercentProgress;
    private boolean failed;
    public TestReportProgress() {
        previousPercentProgress = 0;
        failed = false;
    }

    @Override
    public void apply(String message, Integer percentage) {
        System.out.println(message);
        if (percentage == null) return;
        if (percentage < previousPercentProgress || percentage < 0 || percentage > 100) failed = true;
        previousPercentProgress = percentage;
    }

    /**
     * Returns whether an invalid sequence of progress reports was received.
     * @return whether an invalid sequence of progress reports was received
     */
    public boolean failed() {
        return failed;
    }
}
