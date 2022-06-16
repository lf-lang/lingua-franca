package org.lflang.generator.c;

import static org.lflang.ASTUtils.allActions;
import static org.lflang.ASTUtils.allInputs;
import static org.lflang.ASTUtils.allOutputs;

import org.lflang.ASTUtils;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Action;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.lf.ReactorDecl;

/**
 * Generates C constructor code for a reactor.
 *
 */
public class CConstructorGenerator {
    /**
     * Generate a constructor for the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param federate A federate name, or null to unconditionally generate.
     * @param constructorCode Lines of code previously generated that need to
     *  go into the constructor.
     */
    public static String generateConstructor(
        ReactorDecl reactor,
        FederateInstance federate,
        String constructorCode
    ) {
        var structType = CUtil.selfType(reactor);
        var code = new CodeBuilder();
        code.pr(structType+"* new_"+reactor.getName()+"() {");
        code.indent();
        code.pr(structType+"* self = ("+structType+"*)_lf_new_reactor(sizeof("+structType+"));");
        // Initialize redundant self pointer for backward compatibility
        // (code that references state variables using the syntax `self->name`).
        code.pr("self->self = self;");
        // The use of macros for parameters and state variables
        // carries the risk that users will name parameters or state variables
        // to match the fields of the structs of ports or actions, such as "value"
        // or "is_present". To guard against this, we make the macro expansion work
        // anyway for these structs by including a "self" field in the struct that
        // points right back to the struct.
        for (Action action : allActions(ASTUtils.toDefinition(reactor))) {
            code.pr("self->_lf_" + action.getName() + ".self = &self->_lf_" + action.getName() + ";");
        }
        for (Input input : allInputs(ASTUtils.toDefinition(reactor))) {
            code.pr("self->_lf_default__" + input.getName() + ".self = &self->_lf_default__" + input.getName() + ";");
        }
        for (Output output : allOutputs(ASTUtils.toDefinition(reactor))) {
            code.pr("self->_lf_" + output.getName() + ".self = &self->_lf_" + output.getName() + ";");
        }
        code.pr(constructorCode);
        code.pr("return self;");
        code.unindent();
        code.pr("}");
        return code.toString();
    }
}
