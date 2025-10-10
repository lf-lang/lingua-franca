package org.lflang.diagram.lsp;

import java.util.HashMap;
import java.util.Map;
import org.eclipse.lsp4j.ProgressParams;
import org.eclipse.lsp4j.WorkDoneProgressBegin;
import org.eclipse.lsp4j.WorkDoneProgressCreateParams;
import org.eclipse.lsp4j.WorkDoneProgressEnd;
import org.eclipse.lsp4j.WorkDoneProgressNotification;
import org.eclipse.lsp4j.WorkDoneProgressReport;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.xtext.util.CancelIndicator;

/**
 * A class for reporting the progress of an ongoing task.
 *
 * @author Peter Donovan
 * @ingroup LSP
 */
public class Progress {
  private static int nextToken = 0;
  private static final Map<Integer, Boolean> cancellations = new HashMap<>();

  private final LanguageClient client;
  private final String title;
  private final int token;
  private final boolean cancellable;

  /**
   * Initialize the `Progress` of a task titled `title` that is triggered via `client`.
   *
   * @param client A language client through which a task was triggered.
   * @param title The title of the task.
   * @param cancellable Whether the task tracked by `this` can be cancelled.
   */
  public Progress(LanguageClient client, String title, boolean cancellable) {
    this.client = client;
    this.title = title;
    this.token = nextToken++;
    this.cancellable = cancellable;
    if (cancellable) cancellations.put(token, false);
    client.createProgress(new WorkDoneProgressCreateParams(Either.forRight(token)));
  }

  /** Cancel the task tracked by the `Progress` that has token `token`. */
  public static void cancel(int token) {
    if (cancellations.containsKey(token)) cancellations.put(token, true);
  }

  /**
   * Return the cancel indicator for the task tracked by this `Progress`.
   *
   * @return the cancel indicator for the task tracked by this `Progress`
   */
  public CancelIndicator getCancelIndicator() {
    if (cancellable) return () -> cancellations.get(token);
    return () -> false;
  }

  /** Report that the task tracked by `this` is done. */
  public void begin() {
    WorkDoneProgressBegin begin = new WorkDoneProgressBegin();
    begin.setTitle(title);
    begin.setCancellable(cancellable);
    begin.setPercentage(0);
    notifyProgress(begin);
  }

  /**
   * Report the progress of the task tracked by `this`.
   *
   * @param message A message describing the progress of the task.
   * @param percentage The percentage of completion (0-100).
   */
  public void report(String message, Integer percentage) {
    WorkDoneProgressReport report = new WorkDoneProgressReport();
    report.setMessage(message);
    report.setCancellable(cancellable);
    report.setPercentage(percentage);
    notifyProgress(report);
  }

  /**
   * Mark the task tracked by `this` as terminated.
   *
   * @param message A message describing the outcome of the task.
   */
  public void end(String message) {
    WorkDoneProgressEnd end = new WorkDoneProgressEnd();
    end.setMessage(message);
    notifyProgress(end);
  }

  /**
   * Send the given progress notification to the client.
   *
   * @param notification
   */
  private void notifyProgress(WorkDoneProgressNotification notification) {
    client.notifyProgress(new ProgressParams(Either.forRight(token), Either.forLeft(notification)));
  }
}
