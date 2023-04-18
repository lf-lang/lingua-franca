package org.lflang.generator.c;

import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Reactor;

/**
 * Generates C constructor code for a reactor.
 *
 */
public class CConstructorGenerator {
    /**
     * Generate a constructor for the specified reactor in the specified federate.
     * @param reactor The parsed reactor data structure.
     * @param constructorCode Lines of code previously generated that need to
     *  go into the constructor.
     */
    public static String generateConstructor(
        Reactor reactor,
        String constructorCode
    ) {
        var structType = CUtil.selfType(reactor);
        var code = new CodeBuilder();
        code.pr(structType+"* new_"+CUtil.getName(reactor)+"() {");
        code.indent();
        code.pr(structType+"* self = ("+structType+"*)_lf_new_reactor(sizeof("+structType+"));");
        code.pr(constructorCode);
        code.pr("return self;");
        code.unindent();
        code.pr("}");
        return code.toString();
    }

    public static String generateConstructorPrototype(Reactor reactor) {
        return CUtil.selfType(reactor)+"* new_"+CUtil.getName(reactor)+"();";
    }
}
