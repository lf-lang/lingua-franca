/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.rust;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.eclipse.emf.ecore.EObject;
import org.lflang.MessageReporter;
import org.lflang.TargetProperty.BuildType;

/**
 * Rust-specific part of a {@link org.lflang.TargetConfig}.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
public final class RustTargetConfig {

  /** List of Cargo features of the generated crate to enable. */
  private List<String> cargoFeatures = new ArrayList<>();

  /** Map of Cargo dependency to dependency properties. */
  private Map<String, CargoDependencySpec> cargoDependencies = new HashMap<>();

  /** List of top-level modules, those are absolute paths. */
  private final List<Path> rustTopLevelModules = new ArrayList<>();

  /** Cargo profile, default is debug (corresponds to cargo dev profile). */
  private BuildType profile = BuildType.DEBUG;

  public void setCargoFeatures(List<String> cargoFeatures) {
    this.cargoFeatures = cargoFeatures;
  }

  public void setCargoDependencies(Map<String, CargoDependencySpec> cargoDependencies) {
    this.cargoDependencies = cargoDependencies;
  }

  public void addAndCheckTopLevelModule(Path path, EObject errorOwner, MessageReporter err) {
    String fileName = path.getFileName().toString();
    if (!Files.exists(path)) {
      err.at(errorOwner).error("File not found");
    } else if (Files.isRegularFile(path) && !fileName.endsWith(".rs")) {
      err.at(errorOwner).error("Not a rust file");
    } else if (fileName.equals("main.rs")) {
      err.at(errorOwner).error("Cannot use 'main.rs' as a module name (reserved)");
    } else if (fileName.equals("reactors") || fileName.equals("reactors.rs")) {
      err.at(errorOwner).error("Cannot use 'reactors' as a module name (reserved)");
    } else if (Files.isDirectory(path) && !Files.exists(path.resolve("mod.rs"))) {
      err.at(errorOwner).error("Cannot find module descriptor in directory");
    }
    this.rustTopLevelModules.add(path);
  }

  public List<String> getCargoFeatures() {
    return cargoFeatures;
  }

  /** Returns a map of cargo dependencies. */
  public Map<String, CargoDependencySpec> getCargoDependencies() {
    return cargoDependencies;
  }

  /**
   * Returns the list of top-level module files to include in main.rs. Those files were checked to
   * exists previously.
   */
  public List<Path> getRustTopLevelModules() {
    return rustTopLevelModules;
  }

  /** The build type to use. Corresponds to a Cargo profile. */
  public BuildType getBuildType() {
    return profile;
  }

  /** Set a build profile chosen based on a cmake profile. */
  public void setBuildType(BuildType profile) {
    this.profile = profile;
  }
}
