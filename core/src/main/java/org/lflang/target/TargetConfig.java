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

import static org.lflang.ast.ASTUtils.convertToEmptyListIfNull;

import com.google.gson.JsonObject;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.resource.Resource;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.GeneratorArguments;
import org.lflang.generator.GeneratorUtils;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.FedSetupProperty;
import org.lflang.target.property.FileListProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.TargetProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.util.FileUtil;

/**
 * A class for keeping the current target configuration.
 *
 * <p>Class members of type String are initialized as empty strings, unless otherwise stated.
 *
 * @author Marten Lohstroh
 */
public class TargetConfig {

  /** Error message to use when a target property does not exist in LF syntax. */
  public static final String NOT_IN_LF_SYNTAX_MESSAGE =
      "There is no representation of this property in the LF target property syntax";

  /** The target of this configuration (e.g., C, TypeScript, Python). */
  public final Target target;

  /** Additional sources to add to the compile command if appropriate. */
  public final List<String> compileAdditionalSources = new ArrayList<>();

  /** Map of target properties */
  protected final Map<TargetProperty<?, ?>, Object> properties = new HashMap<>();

  /** Map from */
  protected final Map<TargetProperty<?, ?>, KeyValuePair> keyValuePairs = new HashMap<>();

  /** Set of target properties that have been assigned a value */
  private final Set<TargetProperty<?, ?>> setProperties = new HashSet<>();

  /** The main resource that is under compilation. */
  protected Resource mainResource;

  /**
   * Return mock instance to use for testing, which is not tied to a generator context or a
   * resource.
   */
  public static TargetConfig getMockInstance(Target target) {
    return new TargetConfig(target);
  }

  /**
   * Create a new target configuration based on the given target declaration AST node only.
   *
   * @param target AST node of a target declaration.
   */
  protected TargetConfig(Target target) {
    this.target = target;

    // Register target-specific properties
    target.initialize(this);

    // Register target properties for internal use.
    this.register(FedSetupProperty.INSTANCE);

    // Register general-purpose properties
    this.register(
        FastProperty.INSTANCE,
        LoggingProperty.INSTANCE,
        NoCompileProperty.INSTANCE,
        TimeOutProperty.INSTANCE);
  }

  /**
   * Load configuration from the given resource.
   *
   * @param resource A resource to load from.
   * @param reporter A message reporter for reporting errors and warnings.
   */
  protected void load(Resource resource, MessageReporter reporter) {
    var targetDecl = GeneratorUtils.findTargetDecl(resource);
    var properties = targetDecl.getConfig();

    // Load properties from file
    if (properties != null) {
      List<KeyValuePair> pairs = properties.getPairs();
      pairs.forEach(
          pair -> {
            var p = forName(pair.getName());
            if (p.isPresent()) {
              var property = p.get();
              // Record the pair.
              keyValuePairs.put(property, pair);
              // Only update the config if the pair matches the type.
              if (property.checkType(pair, reporter)) {
                property.update(this, pair, reporter);
                // Ignore properties if they are imported and must not load from imports.
              }
            } else {
              reportUnsupportedTargetProperty(
                  pair.getName(), reporter.at(pair, Literals.KEY_VALUE_PAIR__NAME));
            }
          });
    }
  }

  /**
   * Create a new target configuration based on the given target declaration AST node and the
   * arguments passed to the code generator.
   *
   * @param resource The main resource.
   * @param args The arguments passed to the code generator.
   * @param reporter An error reporter.
   */
  public TargetConfig(Resource resource, GeneratorArguments args, MessageReporter reporter) {
    this(Target.fromDecl(GeneratorUtils.findTargetDecl(resource)));
    this.mainResource = resource;
    load(resource, reporter);
    load(args, reporter);
    // Validate to ensure consistency
    validate(reporter);
  }

  /**
   * If the federate that the target configuration applies to is imported, merge target properties
   * declared in the file that it was imported from.
   *
   * @param importedResource The resource in which the target configuration is to be loaded from.
   * @param mainResource The resource in which the main reactor is specified.
   * @param loadOrNot Predicate to determine for each target property whether it should be loaded.
   * @param messageReporter An error reporter to use when problems are encountered.
   */
  public void mergeImportedConfig(
      Resource importedResource,
      Resource mainResource,
      Predicate<TargetProperty> loadOrNot,
      MessageReporter messageReporter) {
    // If the federate is imported, then update the configuration based on target properties
    // in the imported file.
    if (!importedResource.equals(mainResource)) {
      var importedTargetDecl = GeneratorUtils.findTargetDecl(importedResource);
      var targetProperties = importedTargetDecl.getConfig();
      if (targetProperties != null) {
        // Merge properties
        update(
            this,
            convertToEmptyListIfNull(targetProperties.getPairs()),
            FileUtil.getRelativePath(mainResource, importedResource),
            loadOrNot,
            messageReporter);
      }
    }
  }

  /**
   * Update the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param pairs AST node that holds all the target properties.
   * @param relativePath The path from the main resource to the resource from which the new
   *     properties originate.
   * @param loadOrNot Predicate to determine whether a property should be loaded.
   * @param err Message reporter for errors.
   */
  public void update(
      TargetConfig config,
      List<KeyValuePair> pairs,
      Path relativePath,
      Predicate<TargetProperty> loadOrNot,
      MessageReporter err) {
    pairs.forEach(
        pair -> {
          var p = config.forName(pair.getName());
          if (p.isPresent()) {
            var value = pair.getValue();
            var property = p.get();
            if (property instanceof FileListProperty fileListProperty) {
              var files =
                  ASTUtils.elementToListOfStrings(value).stream()
                      .map(relativePath::resolve) // assume all paths are relative
                      .map(Objects::toString)
                      .toList();
              fileListProperty.update(config, files);
            } else if (loadOrNot.test(property)) {
              p.get().update(this, pair, err);
            }
          }
        });
  }

  /**
   * Update this configuration based on the given JSON object.
   *
   * @param jsonObject The JSON object to read updates from.
   * @param messageReporter A message reporter to report issues.
   */
  protected void load(JsonObject jsonObject, MessageReporter messageReporter) {
    if (jsonObject != null && jsonObject.has("properties")) {
      var map = jsonObject.getAsJsonObject("properties").asMap();
      map.keySet()
          .forEach(
              name -> {
                var property = this.forName(name);
                if (property.isPresent()) {
                  property.get().update(this, map.get(name), messageReporter);
                } else {
                  reportUnsupportedTargetProperty(name, messageReporter.nowhere());
                }
              });
    }
  }

  /** Return `true` if the given target property is supported, `false` otherwise. */
  public boolean isSupported(TargetProperty p) {
    return properties.containsKey(p);
  }

  /**
   * Report that a target property is not supported by the current target.
   *
   * @param name The name of the unsupported target property.
   * @param stage2 The second stage an the error reporter through which to report the warning.
   */
  public void reportUnsupportedTargetProperty(String name, MessageReporter.Stage2 stage2) {
    stage2.error(
        String.format(
            "The target property '%s' is not supported by the %s target.", name, this.target));
    stage2.info("Recognized properties are: " + this.listOfRegisteredProperties());
  }

  /** Get the main resource that is under compilation. */
  public Resource getMainResource() {
    return mainResource;
  }

  /**
   * Register target properties and assign them their initial value.
   *
   * @param properties The target properties to register.
   */
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

  /**
   * Return the value currently assigned to the given target property, or its default value if none
   * been set.
   *
   * @param property The property to get the value of
   * @return The current value, or the initial value of none was assigned.
   * @param <T> The Java type of the returned value.
   * @param <S> The LF type of the returned value.
   */
  public <T, S extends TargetPropertyType> T getOrDefault(TargetProperty<T, S> property) {
    try {
      return get(property);
    } catch (IllegalArgumentException e) {
      return property.initialValue();
    }
  }

  /**
   * Return `true` if this target property has been set (past initialization), `false`
   * otherwise.
   */
  public boolean isSet(TargetProperty<?, ?> property) {
    return this.setProperties.contains(property);
  }

  /** Return the target properties that are currently registered. */
  public String listOfRegisteredProperties() {
    return getRegisteredProperties().stream()
        .map(TargetProperty::toString)
        .filter(s -> !s.startsWith("_"))
        .collect(Collectors.joining(", "));
  }

  /** Return the target properties that have been assigned a value. */
  public List<TargetProperty<?, ?>> getAssignedProperties() {
    return this.properties.keySet().stream()
        .filter(this::isSet)
        .sorted(Comparator.comparing(p -> p.getClass().getName()))
        .collect(Collectors.toList());
  }

  /** Return the target properties that are currently registered. */
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

  /**
   * Load overrides passed in as CLI arguments.
   *
   * @param args List of overrides that may or may not be set
   * @param err Message reporter to report attempts to set unsupported target properties.
   */
  public void load(GeneratorArguments args, MessageReporter err) {
    load(args.jsonObject(), err);
    args.overrides().forEach(a -> a.update(this, err));
  }

  /**
   * Assign the given value to the given target property.
   *
   * @param property The target property to assign the value to.
   * @param value The value to assign to the target property.
   * @param <T> The Java type of the value.
   * @param <S> The LF type of the value.
   */
  public <T, S extends TargetPropertyType> void set(TargetProperty<T, S> property, T value) {
    if (value != null) {
      this.setProperties.add(property);
      this.properties.put(property, value);
    }
  }

  /** Return a description of the current settings. */
  public String settings() {
    var sb = new StringBuffer("Target Configuration:\n");
    this.properties.keySet().stream()
        .filter(this.setProperties::contains)
        .forEach(
            p -> sb.append(String.format("      - %s: %s\n", p.name(), this.get(p).toString())));
    sb.setLength(sb.length() - 1);
    return sb.toString();
  }

  /**
   * Return the AST node that was used to assign a value for the given target property.
   *
   * @param targetProperty The target property to find a matching AST node for.
   * @param <T> The Java type of values assigned to the given target property.
   * @param <S> The LF type of values assigned to the given target property.
   */
  public <T, S extends TargetPropertyType> KeyValuePair lookup(
      TargetProperty<T, S> targetProperty) {
    return this.keyValuePairs.get(targetProperty);
  }

  /** Return whether this configuration is used in the context of a federated program. */
  public boolean isFederated() {
    return ASTUtils.getFederatedReactor(this.getMainResource()).isPresent();
  }

  /**
   * Extract all properties as a list of key-value pairs from a TargetConfig. The returned list only
   * includes properties that were explicitly set.
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
   * Construct a `TargetDecl` by extracting the fields of the given `TargetConfig`.
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
   * Validate all set properties and report issues via the given reporter.
   *
   * @param reporter A reporter to report errors and warnings through.
   */
  public void validate(MessageReporter reporter) {
    this.setProperties.forEach(
        p -> {
          p.validate(this, reporter);
        });
  }
}
