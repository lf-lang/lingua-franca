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

import java.nio.file.Path;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Properties;
import java.util.stream.Collectors;
import org.lflang.AbstractTargetProperty;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.TargetDecl;
import org.lflang.validation.ValidatorMessageReporter;

/**
 * A target properties along with a type and a list of supporting targets that supports it, as well
 * as a function for configuration updates.
 *
 * @author Marten Lohstroh
 */
public class TargetProperty {

  public static void load(TargetConfig config, Properties properties, MessageReporter err) {
    for (Object key : properties.keySet()) {
      var p = config.forName(key.toString());
      if (p.isPresent()) {
        try {
          p.get().set(properties.get(key).toString(), err);
        } catch (InvalidLfSourceException e) {
          err.at(e.getNode()).error(e.getProblem());
        }
      }
    }
  }

  /**
   * Set the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param properties AST node that holds all the target properties.
   * @param err Error reporter on which property format errors will be reported
   */
  public static void load(TargetConfig config, List<KeyValuePair> properties, MessageReporter err) {
    if (properties == null) {
      return;
    }
    properties.forEach(
        property -> {
          var p = config.forName(property.getName());
          if (p.isPresent()) {
            try {
              p.get().set(property.getValue(), err);
            } catch (InvalidLfSourceException e) {
              err.at(e.getNode()).error(e.getProblem());
            }
          }
        });
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
    for (AbstractTargetProperty p : TargetProperty.loaded(config)) {
      KeyValuePair kv = LfFactory.eINSTANCE.createKeyValuePair();
      kv.setName(p.name());
      kv.setValue(p.toAstElement());
      if (kv.getValue() != null) {
        res.add(kv);
      }
    }
    return res;
  }

  /**
   * Return all the target properties that have been set.
   *
   * @param config The configuration to find the properties in.
   */
  public static List<AbstractTargetProperty> loaded(TargetConfig config) {
    return config.getRegisteredProperties().stream()
        .filter(p -> config.isSet(p))
        .collect(Collectors.toList());
  }

  /**
   * Constructs a {@code TargetDecl} by extracting the fields of the given {@code TargetConfig}.
   *
   * @param target The target to generate for.
   * @param config The TargetConfig to extract from.
   * @return A generated TargetDecl.
   */
  public static TargetDecl extractTargetDecl(Target target, TargetConfig config) {
    TargetDecl decl = LfFactory.eINSTANCE.createTargetDecl();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (KeyValuePair p : extractProperties(config)) {
      kvp.getPairs().add(p);
    }
    decl.setName(target.toString());
    decl.setConfig(kvp);
    return decl;
  }

  /**
   * Retrieve a key-value pair from the given AST that matches the given target property.
   *
   * @param ast The AST retrieve the key-value pair from.
   * @param property The target property of interest.
   * @return The found key-value pair, or {@code null} if no matching pair could be found.
   */
  public static KeyValuePair getKeyValuePair(Model ast, AbstractTargetProperty property) {
    var targetProperties = ast.getTarget().getConfig();
    List<KeyValuePair> properties =
        targetProperties.getPairs().stream()
            .filter(pair -> pair.getName().equals(property.name()))
            .toList();
    assert properties.size() <= 1;
    return properties.size() > 0 ? properties.get(0) : null;
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
    pairs.getPairs().stream()
        .forEach(
            pair -> {
              var match =
                  config.getRegisteredProperties().stream()
                      .filter(prop -> prop.name().equalsIgnoreCase(pair.getName()))
                      .findAny();
              if (match.isPresent()) {
                var p = match.get();
                p.checkSupport(pair, config.target, reporter);
                p.checkType(pair, reporter);
                p.validate(pair, ast, reporter);
              } else {
                reporter
                    .at(pair, Literals.KEY_VALUE_PAIR__NAME)
                    .warning(
                        "Unrecognized target property: "
                            + pair.getName()
                            + ". Recognized properties are: "
                            + config.listOfRegisteredProperties());
              }
            });
  }

  /**
   * Update the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param properties AST node that holds all the target properties.
   * @param relativePath The path from the main resource to the resource from which the new
   *     properties originate.
   */
  public static void update(
      TargetConfig config, List<KeyValuePair> properties, Path relativePath, MessageReporter err) {
    properties.forEach(
        property -> {
          var p = config.forName(property.getName());
          if (p.isPresent()) {
            var value = property.getValue();
            if (property.getName().equals("files")) {
              var array = LfFactory.eINSTANCE.createArray();
              ASTUtils.elementToListOfStrings(property.getValue()).stream()
                  .map(relativePath::resolve) // assume all paths are relative
                  .map(Objects::toString)
                  .map(
                      s -> {
                        var element = LfFactory.eINSTANCE.createElement();
                        element.setLiteral(s);
                        return element;
                      })
                  .forEach(array.getElements()::add);
              value = LfFactory.eINSTANCE.createElement();
              value.setArray(array);
            }
            p.get().set(value, err);
          }
        });
  }
}
