package org.lflang.generator.c;

import java.io.IOException;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.lflang.InferredType;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.util.FileUtil;

public class CReactorHeaderFileGenerator {

    public static void doGenerate(CTypes types, Reactor r, CFileConfig fileConfig) throws IOException {
        String contents = generateHeaderFile(types, r);
        FileUtil.writeToFile(contents, fileConfig.getIncludePath().resolve(r.getName() + ".h"));
    }
    private static String generateHeaderFile(CTypes types, Reactor r) {
        CodeBuilder builder = new CodeBuilder();
        appendPoundIncludes(builder);
        appendSelfStruct(builder, types, r);
        for (Reaction reaction : r.getReactions()) {
            appendSignature(builder, types, reaction, r);
        }
        return builder.getCode();
    }
    private static void appendPoundIncludes(CodeBuilder builder) {
        builder.pr("""
            #include "../include/api/api.h"
            #include "../include/core/reactor.h"
        """);
    }

    public static String selfStructName(String name) {
        return name + "_self_t";
    }

    private static void appendSelfStruct(CodeBuilder builder, CTypes types, Reactor r) {
        builder.pr("typedef struct " + selfStructName(r.getName()) + "{");
        for (StateVar s : r.getStateVars()) {
            builder.pr(types.getTargetType(s.getType()) + " " + s.getName() + ";");
        }
        builder.pr("} " + selfStructName(r.getName()) + ";");
    }

    private static void appendSignature(CodeBuilder builder, CTypes types, Reaction r, Reactor reactor) {
        builder.pr("void " + r.getName() + "(" + reactionParameters(types, r, reactor) + ");");
    }

    private static String reactionParameters(CTypes types, Reaction r, Reactor reactor) {
        return Stream.concat(Stream.of(selfStructName(reactor.getName()) + "* self"), inputTypedVariableStream(r)
            .map((tv) -> types.getVariableDeclaration(InferredType.fromAST(tv.getType()), tv.getName(), false)))
            .collect(Collectors.joining(", "));
    }

    private static Stream<TypedVariable> inputTypedVariableStream(Reaction r) {
        return inputVarRefStream(r).map(it -> it.getVariable() instanceof TypedVariable tv ? tv : null)
            .filter(Objects::nonNull);
    }

    private static Stream<VarRef> inputVarRefStream(Reaction reaction) {
        return varRefStream(Stream.concat(reaction.getTriggers().stream(), reaction.getSources().stream()));
    }

    private static Stream<VarRef> triggerVarRefStream(Reaction reaction) {
        return varRefStream(reaction.getTriggers().stream());
    }

    private static Stream<VarRef> varRefStream(Stream<TriggerRef> toFilter) {
        return toFilter.map(it -> it instanceof VarRef v ? v : null)
            .filter(Objects::nonNull);
    }

    private static Stream<VarRef> outputVarRefStream(Reaction reaction) {
        return reaction.getEffects().stream();
    }

    private static Stream<Variable> inputVariableStream(Reaction reaction) {
        return inputVarRefStream(reaction).map(VarRef::getVariable);
    }

    private static String representVarRef(VarRef it, String multiportIndex, String bankIndex) {
        String containerRepresentation = it.getContainer() == null ? null :
            it.getContainer().getName() + (
                it.getContainer().getWidthSpec() == null ? "" : "[" + bankIndex + "]"
            );
        String varRepresentation = it.getVariable().getName() + (
            !(it instanceof Port p) ? "" : p.getWidthSpec() == null ? "" : "[" + multiportIndex + "]"
        );
        return (containerRepresentation == null ? "" : containerRepresentation + ".") + varRepresentation;
    }

    private static String cVariableOfVarRef(VarRef it) {
        return it.getContainer() != null ? it.getContainer().getName() : it.getVariable().getName();
    }

    private static Stream<VarRef> varRefStream(Reaction reaction) {
        return Stream.concat(inputVarRefStream(reaction), outputVarRefStream(reaction));
    }
}
