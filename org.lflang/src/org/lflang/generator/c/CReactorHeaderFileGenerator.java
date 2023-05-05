package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.lflang.ASTUtils;
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
        void generate(CodeBuilder b, Reactor r, boolean userFacing);
    }

    /** Return the path to the user-visible header file that would be generated for {@code r}. */
    public static Path outputPath(Reactor r) {
        return Path.of(Path.of(r.eResource().getURI().toFileString())
                        .getFileName()
                        .toString()
                        .replaceFirst("[.][^.]+$", ""))
                .resolve(r.getName() + ".h");
    }

    /** Generate the user-visible header file for {@code r}. */
    public static void doGenerate(
            CTypes types,
            Reactor r,
            CFileConfig fileConfig,
            GenerateAuxiliaryStructs generator,
            Function<Reactor, String> topLevelPreamble)
            throws IOException {
        String contents = generateHeaderFile(types, r, generator, topLevelPreamble.apply(r));
        FileUtil.writeToFile(contents, fileConfig.getIncludePath().resolve(outputPath(r)));
    }

    private static String generateHeaderFile(
            CTypes types, Reactor r, GenerateAuxiliaryStructs generator, String topLevelPreamble) {
        CodeBuilder builder = new CodeBuilder();
        appendIncludeGuard(builder, r);
        builder.pr(topLevelPreamble);
        appendPoundIncludes(builder);
        appendSelfStruct(builder, types, r);
        generator.generate(builder, r, true);
        for (Reaction reaction : r.getReactions()) {
            appendSignature(builder, reaction, r);
        }
        builder.pr("#endif");
        return builder.getCode();
    }

    private static void appendIncludeGuard(CodeBuilder builder, Reactor r) {
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
            #include "../include/api/set.h"
            #include "../include/core/reactor.h"
            #ifdef __cplusplus
            }
            #endif
            """);
    }

    /** The type name of the user-facing version of the self struct. */
    private static String userFacingSelfType(Reactor r) {
        return r.getName().toLowerCase() + "_self_t";
    }

    private static void appendSelfStruct(CodeBuilder builder, CTypes types, Reactor r) {
        builder.pr("typedef struct " + userFacingSelfType(r) + "{");
        for (Parameter p : r.getParameters()) {
            builder.pr(types.getTargetType(p) + " " + p.getName() + ";");
        }
        for (StateVar s : r.getStateVars()) {
            builder.pr(types.getTargetType(s) + " " + s.getName() + ";");
        }
        builder.pr("int end[0]; // placeholder; MSVC does not compile empty structs");
        builder.pr("} " + userFacingSelfType(r) + ";");
    }

    /** Generate the signature of the reaction function of {@code r}. */
    private static void appendSignature(CodeBuilder builder, Reaction r, Reactor reactor) {
        if (r.getName() != null) builder.pr("void " + r.getName() + "(" + reactionParameters(r, reactor) + ");");
    }
    /** Return a string representation of the parameters of the reaction function of {@code r}. */
    private static String reactionParameters(Reaction r, Reactor reactor) {
        return Stream.concat(
                        Stream.of(userFacingSelfType(reactor) + "* self"),
                        portVariableStream(r, reactor).map(it -> it.getType(true) + " " + it.getName()))
                .collect(Collectors.joining(", "));
    }

    /** Generate initialization code that is needed if {@code r} is not inlined. */
    public static String nonInlineInitialization(Reaction r, Reactor reactor) {
        var mainDef = LfFactory.eINSTANCE.createInstantiation();
        mainDef.setName(reactor.getName());
        mainDef.setReactorClass(ASTUtils.findMainReactor(reactor.eResource()));
        return portVariableStream(r, reactor)
                .map(it -> it.container == null
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
                                                reactor.getInstantiations().stream()
                                                        .filter(instantiation -> ASTUtils.toDefinition(
                                                                        instantiation.getReactorClass())
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
    public static String reactionArguments(Reaction r, Reactor reactor) {
        return Stream.concat(
                        Stream.of(getApiSelfStruct(reactor)),
                        portVariableStream(r, reactor)
                                .map(it -> String.format("((%s) %s)", it.getType(true), it.getAlias())))
                .collect(Collectors.joining(", "));
    }

    /** Return code for extracting the user-facing part of the self struct from the self struct. */
    private static String getApiSelfStruct(Reactor reactor) {
        return "(" + userFacingSelfType(reactor) + "*) (((char*) self) + sizeof(self_base_t))";
    }

    /** Return a stream of all ports referenced by the signature of {@code r}. */
    private static Stream<PortVariable> portVariableStream(Reaction r, Reactor defaultReactorClass) {
        return varRefStream(r)
                .map(it -> it.getVariable() instanceof TypedVariable tv
                        ? new PortVariable(
                                tv,
                                ASTUtils.toDefinition(
                                        it.getContainer() == null
                                                ? defaultReactorClass
                                                : it.getContainer().getReactorClass()),
                                it.getContainer())
                        : null)
                .filter(Objects::nonNull);
    }

    /**
     * A variable that refers to a port.
     * @param tv The variable of the variable reference.
     * @param r The reactor that contains the port.
     * @param container The {@code Instantiation} referenced in the obtaining of {@code tv}, if
     * applicable; {@code null} otherwise.
     */
    private record PortVariable(TypedVariable tv, Reactor r, Instantiation container) {
        String getType(boolean userFacing) {
            var typeName = container == null
                    ? CGenerator.variableStructType(tv, r, userFacing)
                    : CPortGenerator.localPortName(container.getReactorClass(), getName());
            var isMultiport = ASTUtils.isMultiport(ASTUtils.allPorts(r).stream()
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
            return container == null || container.getWidthSpec() == null ? null : "self->_lf_" + r.getName() + "_width";
        }
        /** The representation of this port as used by the LF programmer. */
        String getRvalue() {
            return container == null ? getName() : container.getName() + "." + getName();
        }
    }

    private static Stream<VarRef> inputVarRefStream(Reaction reaction) {
        return varRefStream(Stream.concat(reaction.getTriggers().stream(), reaction.getSources().stream()));
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
