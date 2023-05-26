package lfformat;

import com.diffplug.spotless.FormatterFunc;
import com.diffplug.spotless.FormatterStep;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.Serializable;
import java.io.Writer;
import java.lang.ProcessBuilder.Redirect;
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
  public static class Step implements FormatterStep {
    // The use of the static keyword here is a workaround for serialization difficulties.
    /** The path to the lingua-franca repository. */
    private static Path projectRoot;

    private static Process formatter;
    private static Writer writer;

    private static BufferedReader reader;
    private static BufferedReader error;

    @Override
    public String format(
        @SuppressWarnings("NullableProblems") String rawUnix,
        @SuppressWarnings("NullableProblems") File file)
        throws IOException, InterruptedException {
      StringBuilder output = new StringBuilder();
      getFormatter();
      try {
        writer.write(file.getAbsoluteFile().toString().strip() + "\n");
        writer.flush();
      } catch (IOException e) {
        getFormatter().waitFor();
        error.lines().forEach(System.out::println);
        formatter = null;
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

    private static Process getFormatter() throws IOException {
      if (formatter != null) return formatter;
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
      return formatter;
    }

//    /** Run the formatter on the given file and return the resulting process handle. */
//    private Process runFormatter(File file) throws IOException {
//      // It looks silly to invoke Java from Java, but it is necessary in
//      // order to break the circularity of needing the program to be built
//      // in order for it to be built.
//      var formatter = getFormatter();
//      return formatter;
//    }

    @SuppressWarnings("NullableProblems")
    @Override
    public String getName() {
      return "Lingua Franca formatting step";
    }
  }
}
