package lfformat;

import com.diffplug.spotless.FormatterStep;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.ResourceBundle;

/**
 * {@code LfFormatStep} is used by the Spotless Gradle plugin as a custom formatting step for
 * formatting LF code.
 */
public final class LfFormatStep {
  private LfFormatStep() {}

  /** Return a {@code FormatterStep} for LF code. */
  public static FormatterStep create(File projectRoot) throws IOException {
    Step.projectRoot = projectRoot.toPath();
    return Step.getInstance();
  }

  /** Implement LF-specific formatting functionality. */
  private static class Step implements FormatterStep {
    // The use of the static keyword here is a workaround for serialization difficulties.
    /** The path to the lingua-franca repository. */
    private static Path projectRoot;

    private static Step instance;

    private static Process formatter;
    private static Writer writer;

    private static BufferedReader reader;
    private static BufferedReader error;

    public static Step getInstance() throws IOException {
      if (instance == null) instance = new Step();
      return instance;
    }

    private Step() throws IOException {
      initializeFormatter();
      Runtime.getRuntime()
          .addShutdownHook(
              new Thread(
                  () -> {
                    try {
                      writer.close();
                      formatter.waitFor();
                      reader.close();
                      error.close();
                    } catch (IOException | InterruptedException e) {
                      throw new RuntimeException(e);
                    }
                  }));
    }

    @Override
    public String format(@SuppressWarnings("NullableProblems") String rawUnix, File file)
        throws IOException, InterruptedException {
      StringBuilder output = new StringBuilder();
      try {
        writer.write(file.getAbsoluteFile().toString().strip() + "\n");
        writer.flush();
      } catch (IOException e) {
        formatter.waitFor();
        error.lines().forEach(System.out::println);
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

    private void initializeFormatter() throws IOException {
      final Path lffPath =
          Path.of(
              "org.lflang",
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
    }

    @SuppressWarnings("NullableProblems")
    @Override
    public String getName() {
      return "Lingua Franca formatting step";
    }
  }
}
