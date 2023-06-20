package org.lflang.cli;

import de.cau.cs.kieler.klighd.Klighd;
import de.cau.cs.kieler.klighd.LightDiagramServices;
import de.cau.cs.kieler.klighd.standalone.KlighdStandaloneSetup;
import java.nio.file.Path;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.lf.Model;
import org.lflang.util.FileUtil;
import picocli.CommandLine.Command;

/**
 * Command lin tool for generating diagrams from Lingua Franca programs.
 *
 * @author Christian Menard
 */
@Command(
    name = "lfd",
    // Enable usageHelp (--help) and versionHelp (--version) options.
    mixinStandardHelpOptions = true,
    versionProvider = VersionProvider.class)
public class Lfd extends CliBase {

  @Override
  public void doRun() {
    KlighdStandaloneSetup.initialize();
    Klighd.setStatusManager(
        (status, style) -> {
          switch (status.getSeverity()) {
            case IStatus.ERROR -> {
              reporter.printError(status.getMessage());
              if (status.getException() != null) {
                status.getException().printStackTrace();
              }
            }
            case IStatus.WARNING -> reporter.printWarning(status.getMessage());
            default -> reporter.printInfo(status.getMessage());
          }
        });

    for (Path relativePath : getInputPaths()) {
      Path path = toAbsolutePath(relativePath);
      final Resource resource = getResource(path);
      if (resource == null) {
        reporter.printFatalErrorAndExit(path.toString() + " is not an LF program.");
      }
      final Model model = (Model) resource.getContents().get(0);
      String baseName = FileUtil.nameWithoutExtension(relativePath);
      Path outFile = io.getWd().resolve(baseName + ".svg").toAbsolutePath();
      IStatus status = LightDiagramServices.renderOffScreen(model, "svg", outFile.toString());
      if (!status.isOK()) {
        reporter.printFatalErrorAndExit(status.getMessage());
      }
    }

    reporter.exit();
  }

  /**
   * Main entry point of the diagram tool.
   *
   * @param args CLI arguments
   */
  public static void main(String[] args) {
    main(Io.SYSTEM, args);
  }

  /**
   * Programmatic entry point, with a custom IO.
   *
   * @param io IO streams.
   * @param args Command-line arguments.
   */
  public static void main(Io io, final String... args) {
    cliMain("lfd", Lfd.class, io, args);
  }
}
