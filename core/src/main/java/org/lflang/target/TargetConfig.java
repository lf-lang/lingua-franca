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
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.MessageReporter;
import org.lflang.TargetProperty;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.validation.ValidatorMessageReporter;

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

    // Register target-specific properties
    target.initialize(this);

    // Register general-purpose properties
    this.register(
        FastProperty.INSTANCE,
        LoggingProperty.INSTANCE,
        NoCompileProperty.INSTANCE,
        TimeOutProperty.INSTANCE,
        TracingProperty.INSTANCE);
  }

  public TargetConfig(TargetDecl target, Properties cliArgs, MessageReporter messageReporter) {
    this(Target.fromDecl(target), target.getConfig(), cliArgs, messageReporter);
  }

  /**
   * Create a new target configuration based on the given commandline arguments and target
   * declaration AST node.
   *
   * @param target The target of this configuration.
   * @param properties The key-value pairs that represent the target properties.
   * @param cliArgs Arguments passed on the commandline.
   * @param messageReporter An error reporter to report problems.
   */
  public TargetConfig(
      Target target,
      KeyValuePairs properties,
      Properties cliArgs,
      MessageReporter messageReporter) {
    this(target);
    if (properties != null) {
      List<KeyValuePair> pairs = properties.getPairs();
      this.load(pairs, messageReporter);
    }

    if (cliArgs != null) {
      this.load(cliArgs, messageReporter);
    }
  }

  /** Additional sources to add to the compile command if appropriate. */
  public final List<String> compileAdditionalSources = new ArrayList<>();

  protected final Map<TargetProperty<?, ?>, Object> properties = new HashMap<>();

  private final Set<TargetProperty<?, ?>> setProperties = new HashSet<>();

  public void register(TargetProperty<?, ?>... properties) {
    Arrays.stream(properties)
        .forEach(property -> this.properties.put(property, property.initialValue()));
  }

  /** Reset this target property to its initial value (and mark it as unset). */
  public void reset(TargetProperty<?, ?> property) {
    this.properties.put(property, property.initialValue());
    this.setProperties.remove(property);
  }

  /** Return the value currently assigned to the given target property. */
  @SuppressWarnings("unchecked")
  public <T, S extends TargetPropertyType> T get(TargetProperty<T, S> property)
      throws IllegalArgumentException {
    if (properties.containsKey(property)) {
      var value = properties.get(property);
      return (T) value;
    } else {
      throw new IllegalArgumentException(
          String.format(
              "Attempting to access target property '%s', which is not supported by the %s target.",
              property.name(), this.target));
    }
  }

  public <T, S extends TargetPropertyType> T getOrDefault(TargetProperty<T, S> property) {
    try {
      return get(property);
    } catch (IllegalArgumentException e) {
      return property.initialValue();
    }
  }

  /**
   * Return {@code true} if this target property has been set (past initialization), {@code false}
   * otherwise.
   */
  public boolean isSet(TargetProperty<?, ?> property) {
    return this.setProperties.contains(property);
  }

  public String listOfRegisteredProperties() {
    return getRegisteredProperties().stream()
        .map(TargetProperty::toString)
        .filter(s -> !s.startsWith("_"))
        .collect(Collectors.joining(", "));
  }

  public List<TargetProperty<?, ?>> getRegisteredProperties() {
    return this.properties.keySet().stream()
        .sorted(Comparator.comparing(p -> p.getClass().getName()))
        .collect(Collectors.toList());
  }

  /**
   * Return the target property in this target config that matches the given string.
   *
   * @param name The string to match against.
   */
  public Optional<TargetProperty<?, ?>> forName(String name) {
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
        err.nowhere().warning("Attempting to load unrecognized target property: " + key);
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

  public <T, S extends TargetPropertyType> void set(TargetProperty<T, S> property, T value) {
    this.setProperties.add(property);
    this.properties.put(property, value);
  }

  /**
   * Extracts all properties as a list of key-value pairs from a TargetConfig. Only extracts
   * properties explicitly set by user.
   *
   * @param config The TargetConfig to extract from.
   * @return The extracted properties.
   */
  public static List<KeyValuePair> extractProperties(TargetConfig config) {
    var res = new LinkedList<KeyValuePair>();
    for (TargetProperty<?, ?> p : config.loaded()) {
      KeyValuePair kv = LfFactory.eINSTANCE.createKeyValuePair();
      var element = p.astElementFromConfig(config);
      if (element.isPresent()) {
        kv.setName(p.name());
        kv.setValue(element.get());
        res.add(kv);
      }
    }
    return res;
  }

  /** Return all the target properties that have been set. */
  public List<TargetProperty<?, ?>> loaded() {
    return getRegisteredProperties().stream().filter(this::isSet).collect(Collectors.toList());
  }

  /**
   * Construct a {@code TargetDecl} by extracting the fields of the given {@code TargetConfig}.
   *
   * @return A generated TargetDecl.
   */
  public TargetDecl extractTargetDecl() {
    TargetDecl declaration = LfFactory.eINSTANCE.createTargetDecl();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (KeyValuePair p : extractProperties(this)) {
      kvp.getPairs().add(p);
    }
    declaration.setName(target.toString());
    declaration.setConfig(kvp);
    return declaration;
  }

  /**
   * Validate the given key-value pairs and report issues via the given reporter.
   *
   * @param pairs The key-value pairs to validate.
   * @param ast The root node of the AST from which the key-value pairs were taken.
   * @param config A target configuration used to retrieve the corresponding target properties.
   * @param reporter A reporter to report errors and warnings through.
   */
  public static void validate(
      KeyValuePairs pairs, Model ast, TargetConfig config, ValidatorMessageReporter reporter) {
    pairs
        .getPairs()
        .forEach(
            pair -> {
              var match =
                  config.getRegisteredProperties().stream()
                      .filter(prop -> prop.name().equalsIgnoreCase(pair.getName()))
                      .findAny();
              if (match.isPresent()) {
                var p = match.get();
                p.checkType(pair, reporter);
                p.validate(pair, ast, reporter);
              } else {
                reporter
                    .at(pair, Literals.KEY_VALUE_PAIR__NAME)
                    .warning(
                        String.format(
                            "The target property '%s' is not supported by the %s target and will"
                                + " thus be ignored.",
                            pair.getName(), config.target));
                reporter
                    .at(pair, Literals.KEY_VALUE_PAIR__NAME)
                    .info("Recognized properties are: " + config.listOfRegisteredProperties());
              }
            });
  }
}
