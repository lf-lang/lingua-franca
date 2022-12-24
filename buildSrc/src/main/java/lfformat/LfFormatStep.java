package lfformat;

import com.diffplug.spotless.FormatterStep;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Serializable;
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
  public static FormatterStep create(File projectRoot) {
    Step.projectRoot = projectRoot.toPath();
    return new Step();
  }

  /** Implement LF-specific formatting functionality. */
  public static class Step implements FormatterStep, Serializable {
    // The use of the static keyword here is a workaround for serialization difficulties.
    /** The path to the lingua-franca repository. */
    private static Path projectRoot;

    @Override
    public String format(
        @SuppressWarnings("NullableProblems") String rawUnix,
        @SuppressWarnings("NullableProblems") File file)
        throws IOException, InterruptedException {
      Process p = runFormatter(file);
      StringBuilder output = new StringBuilder();
      BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
      String line;
      while ((line = in.readLine()) != null) {
        output.append(line).append("\n");
      }
      int returnCode = p.waitFor();
      if (returnCode != 0) {
        throw new RuntimeException("Failed to reformat file.");
      }
      return output.toString();
    }

    /** Run the formatter on the given file and return the resulting process handle. */
    private Process runFormatter(File file) throws IOException {
      final Path resourcePath = projectRoot.resolve(Path.of("org.lflang", "src", "org", "lflang"));
      final ResourceBundle properties =
          ResourceBundle.getBundle(
              "StringsBundle",
              Locale.getDefault(),
              new URLClassLoader(new URL[] {resourcePath.toUri().toURL()}));
      final Path lffPath =
          Path.of(
              "org.lflang",
              "build",
              "libs",
              String.format("org.lflang-%s.jar", properties.getString("VERSION")));
      // It looks silly to invoke Java from Java, but it is necessary in
      // order to break the circularity of needing the program to be built
      // in order for it to be built.
      return new ProcessBuilder(
              List.of(
                  "java",
                  "-cp",
                  lffPath.toString(),
                  "org.lflang.cli.Lff",
                  "--dry-run",
                  file.getAbsoluteFile().toString()))
          .start();
    }

    @SuppressWarnings("NullableProblems")
    @Override
    public String getName() {
      return "Lingua Franca formatting step";
    }
  }
}
