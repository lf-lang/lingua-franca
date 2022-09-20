package lfformat;

import com.diffplug.spotless.FormatterStep;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Path;
import java.util.List;

public final class LfFormatStep {
    private LfFormatStep() {}

    public static FormatterStep create() {
        return new Step();
    }

    private static final class Step implements FormatterStep {
        @Override
        public String format(@SuppressWarnings("NullableProblems") String rawUnix, File file)
                throws IOException, InterruptedException {
            // It looks stupid to invoke Java from Java, but it is necessary in
            // order to break the circularity of needing the program to be built
            // in order for it to be built.
            Process p =
                    new ProcessBuilder(
                                    List.of(
                                            "java",
                                            "-jar",
                                            Path.of(
                                                            "org.lflang.cli",
                                                            "build",
                                                            "libs",
                                                            "org.lflang.cli-0.3.1-SNAPSHOT-lff.jar")
                                                    .toString(),
                                            "--dry-run",
                                            file.getAbsoluteFile().toString()))
                            .start();
            StringBuilder output = new StringBuilder();
            BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
            String line;
            while ((line = in.readLine()) != null) {
                if (!output.isEmpty()) output.append("\n");
                // Filtering by "lff: " is yet another string-processing hack that is not airtight!
                if (!line.startsWith("lff: ")) output.append(line);
            }
            int returnCode = p.waitFor();
            if (returnCode != 0) throw new RuntimeException("Failed to reformat file.");
            return output.toString().stripTrailing() + "\n"; // Not sure why this is necessary
        }

        @SuppressWarnings("NullableProblems")
        @Override
        public String getName() {
            return "Lingua Franca formatting step";
        }
    }
}
