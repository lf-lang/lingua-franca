package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.util.FileUtil;

/** Generate user-visible header files. */
public class CReactorHeaderFileGenerator {

  /** Functional interface for generating auxiliary structs such as port structs. */
  public interface GenerateAuxiliaryStructs {
    void generate(CodeBuilder b, TypeParameterizedReactor r, boolean userFacing);
  }

  /** Return the path to the user-visible header file that would be generated for {@code r}. */
  public static Path outputPath(TypeParameterizedReactor tpr) {
    return Path.of(
            FileUtil.toPath(tpr.reactor().eResource().getURI())
                .getFileName()
                .toString()
                .replaceFirst("[.][^.]+$", ""))
        .resolve(tpr.getName() + ".h");
  }

  public static void doGenerate(
      CTypes types,
      TypeParameterizedReactor tpr,
      CFileConfig fileConfig,
      GenerateAuxiliaryStructs generator,
      Function<Reactor, String> topLevelPreamble)
      throws IOException {
    String contents =
        generateHeaderFile(types, tpr, generator, topLevelPreamble.apply(tpr.reactor()));
    FileUtil.writeToFile(contents, fileConfig.getIncludePath().resolve(outputPath(tpr)));
  }

  private static String generateHeaderFile(
      CTypes types,
      TypeParameterizedReactor tpr,
      GenerateAuxiliaryStructs generator,
      String topLevelPreamble) {
    CodeBuilder builder = new CodeBuilder();
    appendIncludeGuard(builder, tpr);
    builder.pr(topLevelPreamble);
    appendPoundIncludes(builder);
    tpr.doDefines(builder);
    appendSelfStruct(builder, types, tpr);
    generator.generate(builder, tpr, true);
    for (Reaction reaction : tpr.reactor().getReactions()) {
      appendSignature(builder, types, reaction, tpr);
    }
    builder.pr("#endif");
    return builder.getCode();
  }

  private static void appendIncludeGuard(CodeBuilder builder, TypeParameterizedReactor r) {
    String macro = CUtil.getName(r) + "_H";
    builder.pr("#ifndef " + macro);
    builder.pr("#define " + macro);
  }

  private static void appendPoundIncludes(CodeBuilder builder) {
    builder.pr(
        """
            #ifdef __cplusplus
            extern "C" {
            #endif
            #include "../include/api/api.h"
            #include "../include/core/reactor.h"
            #ifdef __cplusplus
            }
            #endif
            """);
  }

  private static String userFacingSelfType(TypeParameterizedReactor tpr) {
    return tpr.getName().toLowerCase() + "_self_t";
  }

  private static void appendSelfStruct(
      CodeBuilder builder, CTypes types, TypeParameterizedReactor tpr) {
    builder.pr("typedef struct " + userFacingSelfType(tpr) + "{");
    builder.indent();
    builder.pr("self_base_t base; // This field is only to be used by the runtime, not the user.");
    for (Parameter p : tpr.reactor().getParameters()) {
      builder.pr(types.getTargetType(p) + " " + p.getName() + ";");
    }
    for (StateVar s : tpr.reactor().getStateVars()) {
      builder.pr(types.getTargetType(s) + " " + s.getName() + ";");
    }
    builder.pr("int end[0]; // placeholder; MSVC does not compile empty structs");
    builder.unindent();
    builder.pr("} " + userFacingSelfType(tpr) + ";");
  }

  private static void appendSignature(
      CodeBuilder builder, CTypes types, Reaction r, TypeParameterizedReactor tpr) {
    if (r.getName() != null)
      builder.pr("void " + r.getName() + "(" + reactionParameters(r, tpr) + ");");
  }

  private static String reactionParameters(Reaction r, TypeParameterizedReactor tpr) {
    return Stream.concat(
            Stream.of(userFacingSelfType(tpr) + "* self"),
            portVariableStream(r, tpr).map(tv -> tv.getType(true) + tv.getName()))
        .collect(Collectors.joining(", "));
  }

  private static String getApiSelfStruct(TypeParameterizedReactor tpr) {
    return "(" + userFacingSelfType(tpr) + "*) self";
  }

  /** Generate initialization code that is needed if {@code r} is not inlined. */
  public static String nonInlineInitialization(Reaction r, TypeParameterizedReactor reactor) {
    var mainDef = LfFactory.eINSTANCE.createInstantiation();
    mainDef.setName(reactor.getName());
    mainDef.setReactorClass(ASTUtils.findMainReactor(reactor.reactor().eResource()));
    return portVariableStream(r, reactor)
        .map(
            it ->
                it.container == null
                    ? ""
                    : it.getWidth() == null
                        ? String.format(
                            "%s %s = (%s) %s;",
                            it.getType(false), it.getAlias(), it.getType(false), it.getRvalue())
                        : String.format(
                            """
                    %s %s[%s];
                    for (int i = 0; i < %s; i++) {
                        %s[i] = (%s) self->_lf_%s[i].%s;
                    }
                    """,
                            it.getType(true).replaceFirst("\\*", ""),
                            it.getAlias(),
                            CReactionGenerator.maxContainedReactorBankWidth(
                                reactor.reactor().getInstantiations().stream()
                                    .filter(
                                        instantiation ->
                                            new TypeParameterizedReactor(instantiation, reactor)
                                                .equals(it.r))
                                    .findAny()
                                    .orElseThrow(),
                                null,
                                0,
                                mainDef),
                            "self->_lf_" + it.container.getName() + "_width",
                            it.getAlias(),
                            it.getType(true).replaceFirst("\\*", ""),
                            it.container.getName(),
                            it.getName()))
        .collect(Collectors.joining("\n"));
  }

  /** Return a string representation of the arguments passed to the function for {@code r}. */
  public static String reactionArguments(Reaction r, TypeParameterizedReactor reactor) {
    return Stream.concat(
            Stream.of(getApiSelfStruct(reactor)),
            portVariableStream(r, reactor)
                .map(it -> String.format("((%s) %s)", it.getType(true), it.getAlias())))
        .collect(Collectors.joining(", "));
  }

  /** Return a stream of all ports referenced by the signature of {@code r}. */
  private static Stream<PortVariable> portVariableStream(
      Reaction r, TypeParameterizedReactor reactorOfReaction) {
    return varRefStream(r)
        .map(
            it ->
                it.getVariable() instanceof TypedVariable tv
                    ? new PortVariable(
                        tv,
                        it.getContainer() != null
                            ? new TypeParameterizedReactor(it.getContainer(), reactorOfReaction)
                            : reactorOfReaction,
                        it.getContainer())
                    : null)
        .filter(Objects::nonNull);
  }

  /**
   * A variable that refers to a port.
   *
   * @param tv The variable of the variable reference.
   * @param r The reactor in which the port is being used.
   * @param container The {@code Instantiation} referenced in the obtaining of {@code tv}, if
   *     applicable; {@code null} otherwise.
   */
  private record PortVariable(
      TypedVariable tv, TypeParameterizedReactor r, Instantiation container) {
    String getType(boolean userFacing) {
      var typeName =
          container == null
              ? CGenerator.variableStructType(tv, r, userFacing)
              : CPortGenerator.localPortName(
                  new TypeParameterizedReactor(container, r),
                  container.getReactorClass(),
                  getName());
      var isMultiport =
          ASTUtils.isMultiport(
              ASTUtils.allPorts(r.reactor()).stream()
                  .filter(it -> it.getName().equals(tv.getName()))
                  .findAny()
                  .orElseThrow());
      return typeName + "*" + (getWidth() != null ? "*" : "") + (isMultiport ? "*" : "");
    }
    /** The name of the variable as it appears in the LF source. */
    String getName() {
      return tv.getName();
    }
    /** The alias of the variable that should be used in code generation. */
    String getAlias() {
      return getName(); // TODO: avoid naming conflicts
    }
    /** The width of the container, if applicable. */
    String getWidth() {
      return container == null || container.getWidthSpec() == null
          ? null
          : "self->_lf_" + r.getName() + "_width";
    }
    /** The representation of this port as used by the LF programmer. */
    String getRvalue() {
      return container == null ? getName() : container.getName() + "." + getName();
    }
  }

  private static Stream<VarRef> inputVarRefStream(Reaction reaction) {
    return varRefStream(
        Stream.concat(reaction.getTriggers().stream(), reaction.getSources().stream()));
  }

  private static Stream<VarRef> varRefStream(Stream<TriggerRef> toFilter) {
    return toFilter.map(it -> it instanceof VarRef v ? v : null).filter(Objects::nonNull);
  }

  private static Stream<VarRef> outputVarRefStream(Reaction reaction) {
    return reaction.getEffects().stream();
  }

  private static Stream<VarRef> varRefStream(Reaction reaction) {
    return Stream.concat(inputVarRefStream(reaction), outputVarRefStream(reaction));
  }
}
