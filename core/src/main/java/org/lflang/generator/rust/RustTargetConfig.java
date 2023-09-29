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

import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.BuildTypeProperty.BuildType;
import org.lflang.target.property.CargoDependenciesProperty;
import org.lflang.target.property.CargoFeaturesProperty;
import org.lflang.target.property.RustIncludeProperty;

/**
 * Rust-specific part of a {@link org.lflang.TargetConfig}.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
public final class RustTargetConfig {

  /** List of Cargo features of the generated crate to enable. */
  public final CargoFeaturesProperty cargoFeatures = new CargoFeaturesProperty();

  /** Map of Cargo dependency to dependency properties. */
  public final CargoDependenciesProperty cargoDependencies = new CargoDependenciesProperty();

  /** List of top-level modules, those are absolute paths. */
  public final RustIncludeProperty rustTopLevelModules = new RustIncludeProperty();

  /** Cargo profile, default is debug (corresponds to cargo dev profile). */
  private BuildType profile = BuildTypeProperty.BuildType.DEBUG;

  /** The build type to use. Corresponds to a Cargo profile. */
  public BuildType getBuildType(BuildTypeProperty cmakeBuildType) {
    // FIXME: this is because Rust uses a different default.
    // Can we just use the same?
    if (cmakeBuildType.isSet()) {
      return cmakeBuildType.get();
    }
    return profile;
  }
}
