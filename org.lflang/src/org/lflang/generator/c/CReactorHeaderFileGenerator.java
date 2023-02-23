package org.lflang.generator.c;

import java.io.IOException;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.lflang.InferredType;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.StateVar;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.util.FileUtil;

public class CReactorHeaderFileGenerator {

    public interface GenerateAuxiliaryStructs {
        void generate(CodeBuilder b, Reactor r);
    }

    public static void doGenerate(CTypes types, Reactor r, CFileConfig fileConfig, GenerateAuxiliaryStructs generator) throws IOException {
        String contents = generateHeaderFile(types, r, generator);
        FileUtil.writeToFile(contents, fileConfig.getIncludePath().resolve(r.getName() + ".h"));
    }
    private static String generateHeaderFile(CTypes types, Reactor r, GenerateAuxiliaryStructs generator) {
        CodeBuilder builder = new CodeBuilder();
        appendPoundIncludes(builder);
        appendSelfStruct(builder, types, r);
        generator.generate(builder, r);
        for (Reaction reaction : r.getReactions()) {
            appendSignature(builder, types, reaction, r);
        }
        return builder.getCode();
    }
    private static void appendPoundIncludes(CodeBuilder builder) {
        builder.pr("""
        #include "../include/api/api.h"
        #include "../include/api/set.h"
        #include "../include/core/reactor.h"
        """);
    }

    public static String selfStructName(String name) {
        return name.toLowerCase() + "_self_t";
    }

    private static void appendSelfStruct(CodeBuilder builder, CTypes types, Reactor r) {
        builder.pr("typedef struct " + selfStructName(r.getName()) + "{");
        for (Parameter p : r.getParameters()) {
            builder.pr(types.getTargetType(p.getType()) + " " + p.getName() + ";");
        }
        for (StateVar s : r.getStateVars()) {
            builder.pr(types.getTargetType(s.getType()) + " " + s.getName() + ";");
        }
        builder.pr("int end[0]; // placeholder; MSVC does not compile empty structs");
        builder.pr("} " + selfStructName(r.getName()) + ";");
    }

    private static void appendSignature(CodeBuilder builder, CTypes types, Reaction r, Reactor reactor) {
        builder.pr("void " + r.getName() + "(" + reactionParameters(types, r, reactor) + ");");
    }

    private static String reactionParameters(CTypes types, Reaction r, Reactor reactor) {
        return Stream.concat(Stream.of(selfStructName(reactor.getName()) + "* self"), ioTypedVariableStream(r)
            .map((tv) -> reactor.getName().toLowerCase() + "_" + tv.getName() + "_t* " + tv.getName()))
            .collect(Collectors.joining(", "));
    }

    public static String reactionArguments(CTypes types, Reaction r, Reactor reactor) {
        return Stream.concat(Stream.of(getApiSelfStruct(reactor)), ioTypedVariableStream(r)
                .map(TypedVariable::getName))
            .collect(Collectors.joining(", "));
    }

    private static String getApiSelfStruct(Reactor reactor) {
        return "(" + CReactorHeaderFileGenerator.selfStructName(reactor.getName()) + "*) (((char*) self) + sizeof(self_base_t))";
    }

    private static Stream<TypedVariable> ioTypedVariableStream(Reaction r) {
        return varRefStream(r).map(it -> it.getVariable() instanceof TypedVariable tv ? tv : null)
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
