package org.lflang.generator;

import com.google.gson.JsonObject;
import java.net.URI;
import java.util.List;

/**
 * Arguments to be passed to a code generator.
 *
 * <p>
 *
 * @param clean Whether to clean before building.
 * @param externalRuntimeUri FIXME: change type of
 *     org.lflang.target.property.ExternalRuntimePathProperty to URI
 * @param hierarchicalBin Whether the bin directory should have a flat or hierarchical organization.
 * @param jsonObject Generator arguments and target properties in JSON format.
 * @param lint For enabling or disabling the linting of generated code.
 * @param quiet Whether to suppress output of the target compiler and other commands.
 * @param rti The location of the rti.
 * @param overrides List of arguments that are meant to override target properties
 * @author Marten Lohstroh
 */
public record GeneratorArguments(
    boolean clean,
    URI externalRuntimeUri,
    boolean hierarchicalBin,
    JsonObject jsonObject,
    boolean lint,
    boolean quiet,
    URI rti,
    List<Argument<?>> overrides) {

  /** Return a record with none of the arguments set. */
  public static GeneratorArguments none() {
    return new GeneratorArguments(false, null, false, null, false, false, null, List.of());
  }
}
