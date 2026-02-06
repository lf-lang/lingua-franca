package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.FileConfig;
import org.lflang.target.property.CmakeIncludeProperty;
import org.lflang.target.property.CmakeInitIncludeProperty;
import org.lflang.target.property.FilesProperty;
import org.lflang.target.property.ProtobufsProperty;
import org.lflang.util.FileUtil;

/**
 * A subclass of {@see FileConfig} that extends the base functionality to add support for compiling
 * federated LF programs.
 *
 * @author Soroush Bateni
 * @ingroup Federated
 */
public class FederationFileConfig extends FileConfig {

  public FederationFileConfig(Resource resource, Path srcGenBasePath, boolean useHierarchicalBin)
      throws IOException {
    super(resource, srcGenBasePath, useHierarchicalBin);
  }

  public FederationFileConfig(FileConfig fileConfig) throws IOException {
    super(fileConfig.resource, fileConfig.getSrcGenBasePath(), fileConfig.useHierarchicalBin);
  }

  /**
   * Return the path to the root of a LF project generated on the basis of a federated LF program
   * currently under compilation.
   */
  public Path getGenPath() {
    return srcPkgPath.resolve("fed-gen").resolve(name);
  }

  /** Return the path for storing generated LF sources that jointly constitute a federation. */
  public Path getSrcPath() {
    return getGenPath().resolve("src");
  }

  /**
   * The directory in which to put the generated sources. This takes into account the location of
   * the source file relative to the package root. Specifically, if the source file is x/y/Z.lf
   * relative to the package root, then the generated sources will be put in x/y/Z relative to
   * srcGenBasePath.
   */
  @Override
  public Path getSrcGenPath() {
    return getGenPath().resolve("src-gen");
  }

  /** The directory in which to put a copy of reactor-c for compiling a RTI for this federation. */
  public Path getRtiSrcGenPath() {
    return getSrcGenPath().resolve("RTI");
  }

  /** The path to the RTI binary that is compiled for this federation. */
  public Path getRtiBinPath() {
    return getFedBinPath().resolve("RTI");
  }

  /**
   * Return the path to the root of a LF project generated on the basis of a federated LF program
   * currently under compilation.
   */
  public Path getFedGenPath() {
    return srcPkgPath.resolve("fed-gen").resolve(this.name);
  }

  /** Return the path to the directory in which the executables of compiled federates are stored. */
  public Path getFedBinPath() {
    return getFedGenPath().resolve("bin");
  }

  public Path getSSTPath() {
    return getGenPath().resolve("sst");
  }

  public Path getSSTConfigPath() {
    return getSSTPath().resolve("configs");
  }

  public Path getSSTCredentialsPath() {
    return getSSTPath().resolve("credentials");
  }

  public Path getSSTGraphsPath() {
    return getSSTPath().resolve("graphs");
  }

  public Path getSSTPolicyPath() {
    return getSSTPath().resolve("policy");
  }

  public Path getSSTAuthPath() {
    return getSSTPath().resolve("auth");
  }

  @Override
  public void doClean() throws IOException {
    super.doClean();
    FileUtil.deleteDirectory(this.getFedGenPath());
  }

  /**
   * Relativize target properties that involve paths like files and cmake-include to be relative to
   * the generated .lf file for the federate.
   */
  public void relativizePaths(FederateTargetConfig targetConfig) {
    List.of(
            ProtobufsProperty.INSTANCE,
            FilesProperty.INSTANCE,
            CmakeIncludeProperty.INSTANCE,
            CmakeInitIncludeProperty.INSTANCE)
        .forEach(
            p -> {
              if (targetConfig.isSet(p)) {
                p.override(targetConfig, relativizePathList(targetConfig.get(p)));
              }
            });
  }

  /**
   * Relativize each path in the given list.
   *
   * @param paths The paths to relativize.
   */
  private List<String> relativizePathList(List<String> paths) {
    List<String> tempList = new ArrayList<>();
    paths.forEach(f -> tempList.add(relativizePath(Paths.get(f))));
    return tempList;
  }

  /**
   * Relativize a single path, but only if it points to a local resource in the project (i.e., not
   * on the class path).
   *
   * @param path The path to relativize.
   */
  private String relativizePath(Path path) {
    if (FileUtil.findInPackage(path, this) == null) {
      return String.valueOf(path);
    } else {
      Path resolvedPath = this.srcPath.resolve(path).toAbsolutePath();
      return this.getSrcPath().relativize(resolvedPath).toString();
    }
  }
}
