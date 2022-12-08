package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.ReactorDecl;
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
        for (Variable port : responsePorts(reaction)) {
            builder.pr(String.format(
                """
                if (%s->is_present) {
                    if (!setjmp(self->ra)) context_switch(%s->value.ctx, &top_of_stack);
                    handled_response = true;
                }""",
                port.getName(),
                port.getName()
            ));
        }
        for (Variable trigger : nonResponseTriggers(reaction)) {
            builder.pr(String.format(
                """
                if (%s->is_present) {
                    handled_non_response = true;
                }""",
                trigger.getName()
                ));
        }
        builder.pr(String.format(
            """
            if (handled_non_response || !handled_response) %s(&top_of_stack, self, %s);
            """,
            auxiliaryFunctionName(decl, reactionCount),
            auxiliaryFunctionArgumentList(reaction)
        ));
        return builder.toString();
    }

    public static String generateAuxiliaryFunction(ReactorDecl decl, Reaction reaction, int reactionCount) {
        return String.format(
            """
            void %s(%s) {
            %s
                longjmp(self->ra, 1);
            }
            """,
            auxiliaryFunctionName(decl, reactionCount),
            auxiliaryFunctionParameterList(decl, reaction),
            ASTUtils.toText(reaction.getCode()).indent(4)
        );
    }

    private static List<Variable> responsePorts(Reaction reaction) {
        return inputVariableStream(reaction)
            .filter(it -> it instanceof Port p && p.getType().isRequest())
            .toList();
    }

    private static List<Variable> nonResponseTriggers(Reaction reaction) {
        return inputVariableStream(reaction)
            .filter(it -> it instanceof Port p && !p.getType().isRequest() || !(it instanceof Port))
            .toList();
    }

    private static String auxiliaryFunctionName(ReactorDecl decl, int reactionCount) {
        return CReactionGenerator.generateReactionFunctionName(decl, reactionCount) + "_helper";
    }

    private static String auxiliaryFunctionArgumentList(Reaction reaction) {
        return variableStream(reaction).map(Variable::getName).collect(Collectors.joining(", "));
    }

    private static String auxiliaryFunctionParameterList(ReactorDecl decl, Reaction reaction) {
        List<String> parameterTypes = new ArrayList<>();
        List<String> parameterNames = new ArrayList<>();
        parameterTypes.add("long*");
        parameterNames.add("top_of_stack");
        parameterTypes.add(CUtil.selfType(decl) + "*");
        parameterNames.add("self");
        variableStream(reaction)
            .map(it -> CGenerator.variableStructType(it, decl))
            .map(it -> it + "*")
            .forEach(parameterTypes::add);
        variableStream(reaction)
            .map(Variable::getName)
            .forEach(parameterNames::add);
        List<String> typesAndNames = new ArrayList<>();
        for (int i = 0; i < parameterNames.size(); i++) {
            typesAndNames.add(parameterTypes.get(i) + " " + parameterNames.get(i));
        }
        return String.join(", ", typesAndNames);
    }

    private static Stream<Variable> inputVariableStream(Reaction reaction) {
        return Stream.concat(reaction.getTriggers().stream(), reaction.getSources().stream())
            .map(it -> it instanceof VarRef v ? v : null)
            .filter(Objects::nonNull)
            .map(VarRef::getVariable);
    }

    private static Stream<Variable> outputVariableStream(Reaction reaction) {
        return reaction.getEffects().stream().map(VarRef::getVariable);
    }

    private static Stream<Variable> variableStream(Reaction reaction) {
        return Stream.concat(inputVariableStream(reaction), outputVariableStream(reaction));
    }
}
