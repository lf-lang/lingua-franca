package org.lflang.diagram.lsp;

import java.util.HashMap;
import java.util.Map;

import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.ProgressParams;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.WorkDoneProgressCreateParams;
import org.eclipse.lsp4j.WorkDoneProgressBegin;
import org.eclipse.lsp4j.WorkDoneProgressEnd;
import org.eclipse.lsp4j.WorkDoneProgressNotification;
import org.eclipse.lsp4j.WorkDoneProgressReport;
import org.eclipse.xtext.util.CancelIndicator;

/**
 * Reports the progress of an ongoing task.
 */
public class Progress {
    private static int nextToken = 0;
    private static final Map<Integer, Boolean> cancellations = new HashMap<>();
    private final String title;
    private final int token;
    private final LanguageClient client;

    /**
     * Initializes the {@code Progress} of a task titled {@code title} that is
     * triggered via {@code client}.
     * @param client A language client through which a task was triggered.
     * @param title The title of the task.
     */
    public Progress(LanguageClient client, String title) {
        this.client = client;
        this.token = nextToken++;
        cancellations.put(token, false);
        this.title = title;
        client.createProgress(new WorkDoneProgressCreateParams(Either.forRight(token)));
    }

    /**
     * Cancels the task tracked by the {@code Progress} that has token
     * {@code token}.
     */
    public static void cancel(int token) {
        cancellations.put(token, true);
    }

    /**
     * Returns the cancel indicator for the task tracked by this
     * {@code Progress}.
     * @return the cancel indicator for the task tracked by this
     * {@code Progress}
     */
    public CancelIndicator getCancelIndicator() {
        return () -> cancellations.get(token);
    }

    /**
     * Reports that the task tracked by {@code this} is done.
     */
    public void begin() {
        WorkDoneProgressBegin begin = new WorkDoneProgressBegin();
        begin.setTitle(title);
        begin.setCancellable(true);
        begin.setPercentage(0);
        notifyProgress(begin);
    }

    /**
     * Reports the progress of the task tracked by {@code this}.
     * @param message A message describing the progress of the task.
     */
    public void report(String message, Integer percentage) {
        WorkDoneProgressReport report = new WorkDoneProgressReport();
        report.setMessage(message);
        report.setCancellable(true);
        report.setPercentage(percentage);
        notifyProgress(report);
    }

    /**
     * Marks the task tracked by {@code this} as terminated.
     * @param message A message describing the outcome of the task.
     */
    public void end(String message) {
        WorkDoneProgressEnd end = new WorkDoneProgressEnd();
        end.setMessage(message);
        notifyProgress(end);
    }

    /**
     * Sends the given progress notification to the client.
     * @param notification
     */
    private void notifyProgress(WorkDoneProgressNotification notification) {
        client.notifyProgress(new ProgressParams(Either.forRight(token), notification));
    }
}
