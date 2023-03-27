package org.lflang.generator.c;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.emf.ecore.EObject;

import org.lflang.ASTUtils;
import org.lflang.FileConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.StateVar;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.TypedVariable;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.WidthSpec;
import org.lflang.util.FileUtil;

public class CReactorHeaderFileGenerator {

    public interface GenerateAuxiliaryStructs {
        void generate(CodeBuilder b, Reactor r, boolean userFacing);
    }

    public static Path outputPath(CFileConfig fileConfig, Reactor r) {
        return Path.of(Path.of(r.eResource().getURI().toFileString())
                .getFileName().toString().replaceFirst("[.][^.]+$", ""))
            .resolve(r.getName() + ".h");
    }

    public static void doGenerate(CTypes types, Reactor r, CFileConfig fileConfig, GenerateAuxiliaryStructs generator, Function<Reactor, String> topLevelPreamble) throws IOException {
        String contents = generateHeaderFile(types, fileConfig, r, generator, topLevelPreamble.apply(r));
        FileUtil.writeToFile(contents, fileConfig.getIncludePath().resolve(outputPath(fileConfig, r)));
    }
    private static String generateHeaderFile(CTypes types, CFileConfig fileConfig, Reactor r, GenerateAuxiliaryStructs generator, String topLevelPreamble) {
        CodeBuilder builder = new CodeBuilder();
        appendIncludeGuard(builder, r);
        builder.pr(topLevelPreamble);
        appendPoundIncludes(builder, r, fileConfig);
        appendSelfStruct(builder, types, r);
        generator.generate(builder, r, true);
        for (Reaction reaction : r.getReactions()) {
            appendSignature(builder, types, reaction, r);
        }
        builder.pr("#endif");
        return builder.getCode();
    }

    private static void appendIncludeGuard(CodeBuilder builder, Reactor r) {
        String macro = CUtil.getName(r) + "_H";
        builder.pr("#ifndef " + macro);
        builder.pr("#define " + macro);
    }
    private static void appendPoundIncludes(CodeBuilder builder, Reactor r, CFileConfig fileConfig) {
        builder.pr("""
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

    private static void appendSignature(CodeBuilder builder, CTypes types, Reaction r, Reactor reactor) {
        if (r.getName() != null) builder.pr("void " + r.getName() + "(" + reactionParameters(types, r, reactor) + ");");
    }

    private static String reactionParameters(CTypes types, Reaction r, Reactor reactor) {
        return Stream.concat(Stream.of(userFacingSelfType(reactor) + "* self"), ioTypedVariableStream(r, reactor)
            .map(it -> it.getType(true) + " " + it.getName()))
            .collect(Collectors.joining(", "));
    }

    public static String nonInlineInitialization(CTypes types, Reaction r, Reactor reactor) {
        var mainDef = LfFactory.eINSTANCE.createInstantiation();
        mainDef.setName(reactor.getName());
        mainDef.setReactorClass(ASTUtils.findMainReactor(reactor.eResource()));
        return ioTypedVariableStream(r, reactor)
            .map(it -> it.getWidth() == null ?
                String.format("%s %s = %s;", it.getType(false), it.getAlias(), it.getRvalue())
                : String.format("""
                    %s %s[%s];
                    for (int i = 0; i < %s; i++) {
                        %s[i] = (%s) self->_lf_%s[i].%s;
                    }
                    """,
                    it.getType(true).replaceFirst("\\*", ""),
                    it.getAlias(),
                    CReactionGenerator.maxContainedReactorBankWidth(
                        reactor.getInstantiations().stream()
                            .filter(instantiation -> ASTUtils.toDefinition(instantiation.getReactorClass()).equals(it.r))
                            .findAny().orElseThrow(),
                        null, 0, mainDef),
                    "self->_lf_"+it.container.getName()+"_width",
                    it.getAlias(),
                    it.getType(true).replaceFirst("\\*", ""),
                    it.container.getName(),
                    it.getName()))
            .collect(Collectors.joining("\n"));
    }

    public static String reactionArguments(CTypes types, Reaction r, Reactor reactor) {
        return Stream.concat(Stream.of(getApiSelfStruct(reactor)), ioTypedVariableStream(r, reactor)
                .map(it -> String.format("((%s) %s)", it.getType(true), it.getAlias())))
            .collect(Collectors.joining(", "));
    }

    private static String getApiSelfStruct(Reactor reactor) {
        return "(" + userFacingSelfType(reactor) + "*) (((char*) self) + sizeof(self_base_t))";
    }

    private static Stream<PortVariable> ioTypedVariableStream(Reaction r, Reactor defaultReactorClass) {
        return varRefStream(r)
            .map(it -> it.getVariable() instanceof TypedVariable tv ?
                new PortVariable(
                    tv,
                    ASTUtils.toDefinition(it.getContainer() == null ? defaultReactorClass : it.getContainer().getReactorClass()),
                    it.getContainer())
                : null)
            .filter(Objects::nonNull);
    }

    private record PortVariable(TypedVariable tv, Reactor r, Instantiation container) {
        String getType(boolean userFacing) {
            var typeName = container == null ?
                CGenerator.variableStructType(tv, r, userFacing)
                : CPortGenerator.localPortName(container.getReactorClass(), getName());
            return typeName + "*" + (getWidth() != null ? "*" : "");
        }
        String getName() {
            return tv.getName();
        }
        String getAlias() {
            return getName();
        }
        String getWidth() {
            return container.getWidthSpec() == null ? null : "self->_lf_"+r.getName()+"_width";
        }
        String getRvalue() {
            return container == null ? getName() : container.getName() + "." + getName();
        }
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
