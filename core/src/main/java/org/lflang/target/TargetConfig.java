/*************
 * Copyright (c) 2019, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.target;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CargoDependenciesProperty;
import org.lflang.target.property.CargoFeaturesProperty;
import org.lflang.target.property.ClockSyncModeProperty;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.CmakeIncludeProperty;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.CompilerFlagsProperty;
import org.lflang.target.property.CompilerProperty;
import org.lflang.target.property.CoordinationOptionsProperty;
import org.lflang.target.property.CoordinationProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.ExportDependencyGraphProperty;
import org.lflang.target.property.ExportToYamlProperty;
import org.lflang.target.property.ExternalRuntimePathProperty;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.FedSetupProperty;
import org.lflang.target.property.FilesProperty;
import org.lflang.target.property.HierarchicalBinProperty;
import org.lflang.target.property.KeepaliveProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.NoRuntimeValidationProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.PrintStatisticsProperty;
import org.lflang.target.property.ProtobufsProperty;
import org.lflang.target.property.Ros2DependenciesProperty;
import org.lflang.target.property.Ros2Property;
import org.lflang.target.property.RuntimeVersionProperty;
import org.lflang.target.property.RustIncludeProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SingleFileProjectProperty;
import org.lflang.target.property.ThreadingProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.VerifyProperty;

/**
 * A class for keeping the current target configuration.
 *
 * <p>Class members of type String are initialized as empty strings, unless otherwise stated.
 *
 * @author Marten Lohstroh
 */
public class TargetConfig {

  /** The target of this configuration (e.g., C, TypeScript, Python). */
  public final Target target;

  /**
   * Create a new target configuration based on the given target declaration AST node only.
   *
   * @param target AST node of a target declaration.
   */
  public TargetConfig(Target target) {
    this.target = target;

    this.register(
        new AuthProperty(),
        new BuildCommandsProperty(),
        new BuildTypeProperty(),
        new ClockSyncModeProperty(),
        new ClockSyncOptionsProperty(),
        new CmakeIncludeProperty(),
        new CompileDefinitionsProperty(),
        new CompilerFlagsProperty(),
        new CompilerProperty(),
        new CoordinationOptionsProperty(),
        new CoordinationProperty(),
        new DockerProperty(),
        new ExportDependencyGraphProperty(),
        new ExportToYamlProperty(),
        new ExternalRuntimePathProperty(),
        new FastProperty(),
        new FilesProperty(),
        new HierarchicalBinProperty(),
        new KeepaliveProperty(),
        new LoggingProperty(),
        new NoCompileProperty(),
        new NoRuntimeValidationProperty(),
        new PlatformProperty(),
        new PrintStatisticsProperty(),
        new ProtobufsProperty(),
        new Ros2DependenciesProperty(),
        new Ros2Property(),
        new RuntimeVersionProperty(),
        new SchedulerProperty(),
        new SingleFileProjectProperty(),
        new ThreadingProperty(),
        new TimeOutProperty(),
        new TracingProperty(),
        new VerifyProperty(),
        new WorkersProperty());

    this.register(new FedSetupProperty());

    this.register(
        new CargoFeaturesProperty(), new CargoDependenciesProperty(), new RustIncludeProperty());
  }

  /**
   * Create a new target configuration based on the given commandline arguments and target
   * declaration AST node.
   *
   * @param cliArgs Arguments passed on the commandline.
   * @param target AST node of a target declaration.
   * @param messageReporter An error reporter to report problems.
   */
  public TargetConfig(Properties cliArgs, TargetDecl target, MessageReporter messageReporter) {
    this(Target.fromDecl(target));
    if (target.getConfig() != null) {
      List<KeyValuePair> pairs = target.getConfig().getPairs();
      this.load(pairs, messageReporter);
    }

    if (cliArgs != null) {
      this.load(cliArgs, messageReporter);
    }
  }

  /** Additional sources to add to the compile command if appropriate. */
  public final List<String> compileAdditionalSources = new ArrayList<>();

  /** Flags to pass to the linker, unless a build command has been specified. */
  public String linkerFlags = "";

  protected final Map<AbstractTargetProperty<?, ?>, Object> properties = new HashMap<>();

  private final Set<AbstractTargetProperty<?, ?>> setProperties = new HashSet<>();

  public void register(AbstractTargetProperty<?, ?>... properties) {
    Arrays.stream(properties)
        .forEach(property -> this.properties.put(property, property.initialValue()));
  }

  /**
   * Manually override the value of this target property.
   *
   * @param value The value to assign to this target property.
   */
  public <T, S extends TargetPropertyType> void override(
      AbstractTargetProperty<T, S> property, T value) {
    this.setProperties.add(property);
    this.properties.put(property, value);
  }

  /** Reset this target property to its initial value (and mark it as unset). */
  public void reset(AbstractTargetProperty<?, ?> property) {
    this.properties.put(property, property.initialValue());
    this.setProperties.remove(property);
  }

  /** Return the value currently assigned to the given target property. */
  @SuppressWarnings("unchecked")
  public <T, S extends TargetPropertyType> T get(AbstractTargetProperty<T, S> property) {
    return (T) properties.get(property);
  }

  /**
   * Return {@code true} if this target property has been set (past initialization), {@code false}
   * otherwise.
   */
  public boolean isSet(AbstractTargetProperty<?, ?> property) {
    return this.setProperties.contains(property);
  }

  public String listOfRegisteredProperties() {
    return getRegisteredProperties().stream()
        .map(p -> p.toString())
        .filter(s -> !s.startsWith("_"))
        .collect(Collectors.joining(", "));
  }

  public List<AbstractTargetProperty<?, ?>> getRegisteredProperties() {
    return this.properties.keySet().stream()
        .sorted((p1, p2) -> p1.getClass().getName().compareTo(p2.getClass().getName()))
        .collect(Collectors.toList());
  }

  /**
   * Return the target property in this target config that matches the given string.
   *
   * @param name The string to match against.
   */
  public Optional<AbstractTargetProperty<?, ?>> forName(String name) {
    return this.getRegisteredProperties().stream()
        .filter(c -> c.name().equalsIgnoreCase(name))
        .findFirst();
  }

  public void load(Properties properties, MessageReporter err) {
    for (Object key : properties.keySet()) {
      var p = this.forName(key.toString());
      if (p.isPresent()) {
        var property = p.get();
        property.update(this, (String) properties.get(key), err);
      } else {
        throw new RuntimeException("Attempting to load unrecognized target property: " + key);
      }
    }
  }

  /**
   * Set the configuration using the given pairs from the AST.
   *
   * @param pairs AST node that holds all the target properties.
   * @param err Error reporter on which property format errors will be reported
   */
  public void load(List<KeyValuePair> pairs, MessageReporter err) {
    if (pairs == null) {
      return;
    }
    pairs.forEach(
        pair -> {
          var p = forName(pair.getName());
          if (p.isPresent()) {
            var property = p.get();
            property.update(this, pair.getValue(), err);
          }
        });
  }

  public <T, S extends TargetPropertyType> void set(
      AbstractTargetProperty<T, S> property, T value) {
    this.setProperties.add(property);
    this.properties.put(property, value);
  }

  public void markSet(AbstractTargetProperty property) {
    this.setProperties.add(property);
  }
}
