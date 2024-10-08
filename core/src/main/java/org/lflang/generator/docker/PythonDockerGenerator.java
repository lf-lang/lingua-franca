package org.lflang.generator.docker;

import java.util.List;
import org.lflang.generator.LFGeneratorContext;

/**
 * Generates the docker file related code for the Python target.
 *
 * @author Hou Seng Wong
 */
public class PythonDockerGenerator extends CDockerGenerator {
  public static final String DEFAULT_BASE_IMAGE = "python:3.10-alpine";

  public PythonDockerGenerator(LFGeneratorContext context) {
    super(context);
  }

  @Override
  public String defaultImage() {
    return DEFAULT_BASE_IMAGE;
  }

  @Override
  protected String generateCopyOfExecutable() {
    var lfModuleName = context.getFileConfig().name;
    return String.join(
        "\n",
        super.generateCopyOfExecutable(),
        "COPY --from=builder /lingua-franca/%s/src-gen ./src-gen".formatted(lfModuleName));
  }

  @Override
  public List<String> defaultEntryPoint() {
    return List.of("python3", "-u", "src-gen/" + context.getFileConfig().name + ".py");
  }
}
