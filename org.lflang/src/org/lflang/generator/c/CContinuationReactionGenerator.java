package org.lflang.generator.c;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Stream;

import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

public class CContinuationReactionGenerator {
    public static boolean isContinuationReaction(Reaction reaction) {
        return reaction.getCode().toString().contains("lf_call");
//        return inputVariableStream(reaction).anyMatch(it -> it instanceof Port p && p.getType().isRequest())
//            && outputVariableStream(reaction).anyMatch(it -> it instanceof Port p && p.getType().isRequest());
    }

    public static String generateCoordinationBody(ReactorDecl decl, Reaction reaction, int reactionCount) {
        // TODO: Do not keep reactionCount around as a loop counter. Refactor that pattern out.
        CodeBuilder builder = new CodeBuilder();
        builder.pr("""
            // This is above the section of the stack that must be copied. A little unnecessary data
            //  will probably copied from the current stack frame in addition, depending on what is
            //  and is not in registers; that's a performance issue, but it's OK for now.
            long top_of_stack;
            // Note that because we overwrite the stack from top_of_stack onward, whether any
            // variables remain the same after invoking context_switch depends on what is on the
            // stack vs. the registers (which are saved in self->ra).
            bool handled_response = false;
            bool handled_non_response = false;
        """);
        for (VarRef port : responsePorts(reaction)) {
            builder.pr(iterateOverPorts(port, portRef -> String.format(
                """
                if (%s->is_present) {
                    if (!setjmp(self->ra)) context_switch(%s->value.ctx, &top_of_stack);
                    handled_response = true;
                }""",
                portRef,
                portRef
            )));
        }
        for (VarRef port : nonResponseTriggeringPorts(reaction)) {
            builder.pr(iterateOverPorts(port, portRef -> String.format(
                """
                if (%s->is_present) {
                    handled_non_response = true;
                }""",
                portRef
            )));
        }
        builder.pr(String.format(
            """
            if (handled_non_response || !handled_response) %s(&top_of_stack, %s);
            """,
            auxiliaryFunctionName(decl, reactionCount),
            auxiliaryFunctionArgumentList(reaction)
        ));
        return builder.toString();
    }

    public static String generateAuxiliaryFunction(String init, ReactorDecl decl, Reaction reaction, int reactionCount) {
        return String.format(
            """
            __attribute__ ((noinline)) // FIXME: SUPPORT COMPILERS OTHER THAN CLANG/GCC https://stackoverflow.com/questions/1474030/how-can-i-tell-gcc-not-to-inline-a-function
            void %s(%s) {
            %s
            %s
                longjmp(self->ra, 1);
            }
            """,
            auxiliaryFunctionName(decl, reactionCount),
            auxiliaryFunctionParameterList(decl, reaction),
            init.indent(4),
            ASTUtils.toText(reaction.getCode()).indent(4)
        );
    }

    private static String iterateOverPorts(VarRef port, Function<String, String> getLoopBody) {
        String loopBody = getLoopBody.apply(representVarRef(port, "i", "j"));
        if (port.getVariable() instanceof Port p && p.getWidthSpec() != null) {
            loopBody = String.format(
                """
                for (int i = 0; i < %s; i++) {
                %s
                }
                """,
                ASTUtils.width(p.getWidthSpec(), List.of(port.getContainer())),
                loopBody.indent(4)
            );
        }
        if (port.getContainer().getWidthSpec() != null) {
            loopBody = String.format(
                """
                for (int j = 0; j < %s; j++) {
                %s
                }
                """,
                CUtil.generateWidthVariable(port.getContainer().getName()),
                loopBody.indent(4)
            );
        }
        return loopBody;
    }

    private static List<VarRef> responsePorts(Reaction reaction) {
        return inputVarRefStream(reaction)
            .filter(it -> it.getVariable() instanceof Port p && p.getType().isRequest())
            .toList();
    }

    private static List<VarRef> nonResponseTriggeringPorts(Reaction reaction) {
        return triggerVarRefStream(reaction)
            .filter(it -> it.getVariable() instanceof Port p && !p.getType().isRequest() || !(it.getVariable() instanceof Port))
            .toList();
    }

    private static String auxiliaryFunctionName(ReactorDecl decl, int reactionCount) {
        return CReactionGenerator.generateReactionFunctionName(decl, reactionCount) + "_helper";
    }

    private static String auxiliaryFunctionArgumentList(Reaction reaction) {
        return "instance_args";
    }

    private static String auxiliaryFunctionParameterList(ReactorDecl decl, Reaction reaction) {
        return "long* top_of_stack, void* instance_args";
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
