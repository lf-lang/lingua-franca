package lfformat;

import com.diffplug.spotless.FormatterStep;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.file.Path;
import java.util.List;

/**
 * {@code LfFormatStep} is used by the Spotless Gradle plugin as a custom formatting step for
 * formatting LF code.
 */
public final class LfFormatStep {
  private LfFormatStep() {}

  /** Return a {@code FormatterStep} for LF code. */
  public static FormatterStep create() throws IOException, InterruptedException {
    return new Step();
  }

  /** Implement LF-specific formatting functionality. */
  private static class Step implements FormatterStep {
    // The use of the static keyword here is a workaround for serialization difficulties.
    /** The path to the lingua-franca repository. */

    private static Process formatter;
    private static Writer writer;

    private static BufferedReader reader;
    private static BufferedReader error;

    private Step() throws IOException, InterruptedException {
      terminateFormatter();
    }

    @Override
    public String format(@SuppressWarnings("NullableProblems") String rawUnix, File file)
        throws IOException, InterruptedException {
      initializeFormatter();
      StringBuilder output = new StringBuilder();
      try {
        writer.write(file.getAbsoluteFile().toString().strip() + "\n");
        writer.flush();
      } catch (IOException e) {
        formatter.waitFor();
        error.lines().forEach(System.out::println);
        formatter = null;
        initializeFormatter();
        throw new RuntimeException("Failed to format " + file + ".\nPlease ensure that this file passes validator checks.");
      }
      String line = reader.readLine();
      while (line != null && !line.endsWith("\0")) {
        if (!output.isEmpty()) output.append("\n");
        output.append(line);
        line = reader.readLine();
      }
      return output.toString();
    }

    /** Idempotently initialize the formatter. */
    private static void initializeFormatter() throws IOException {
      if (formatter != null) return;
      final Path lffPath =
          Path.of(
              "cli",
              "lff",
              "build",
              "install",
              "lff",
              "bin",
              "lff");
      formatter = new ProcessBuilder(
          List.of(
              lffPath.toString(),
              "--dry-run",
              "--stdin"))
          .start();
      writer = new BufferedWriter(new OutputStreamWriter(formatter.getOutputStream()));
      reader = new BufferedReader(new InputStreamReader(formatter.getInputStream()));
      error = new BufferedReader(new InputStreamReader(formatter.getErrorStream()));
      Runtime.getRuntime()
          .addShutdownHook(
              new Thread(
                  () -> {
                    try {
                      terminateFormatter();
                    } catch (IOException | InterruptedException e) {
                      throw new RuntimeException(e);
                    }
                  }));
    }

    /** Idempotently trigger a graceful termination of the formatter. */
    private static void terminateFormatter() throws IOException, InterruptedException {
      if (formatter == null) return;
      writer.close();
      formatter.waitFor();
      reader.close();
      error.close();
      formatter = null;
    }

    @SuppressWarnings("NullableProblems")
    @Override
    public String getName() {
      return "Lingua Franca formatting step";
    }
  }
}
