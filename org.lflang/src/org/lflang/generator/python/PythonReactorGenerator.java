package org.lflang.generator.python;

import java.util.ArrayList;
import java.util.List;
import org.lflang.ASTUtils;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.DelayBodyGenerator;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;

public class PythonReactorGenerator {
    /**
     * Wrapper function for the more elaborate generatePythonReactorClass that keeps track
     * of visited reactors to avoid duplicate generation
     * @param instance The reactor instance to be generated
     */
    public static String generatePythonClass(ReactorInstance instance, ReactorInstance main, PythonTypes types) {
        List<String> instantiatedClasses = new ArrayList<>();
        return generatePythonClass(instance, instantiatedClasses, main, types);
    }

    /**
     * Generate a Python class corresponding to decl
     * @param instance The reactor instance to be generated
     * @param instantiatedClasses A list of visited instances to avoid generating duplicates
     */
    public static String generatePythonClass(ReactorInstance instance,
                                           List<String> instantiatedClasses,
                                           ReactorInstance main, PythonTypes types) {
        CodeBuilder pythonClasses = new CodeBuilder();
        ReactorDecl decl = instance.getDefinition().getReactorClass();
        Reactor reactor = ASTUtils.toDefinition(decl);
        String className = PyUtil.getName(decl);
        if (instantiatedClasses == null) {
            return "";
        }

        if (!instantiatedClasses.contains(className)) {
            pythonClasses.pr(generatePythonClassHeader(className));
            // Generate preamble code
            pythonClasses.indent();
            pythonClasses.pr(PythonPreambleGenerator.generatePythonPreambles(reactor.getPreambles()));
            // Handle runtime initializations
            pythonClasses.pr(generatePythonConstructor(decl, types));
            pythonClasses.pr(PythonParameterGenerator.generatePythonGetters(decl));
            // Generate methods
            pythonClasses.pr(PythonMethodGenerator.generateMethods(reactor));
            // Generate reactions
            List<Reaction> reactionToGenerate = ASTUtils.allReactions(reactor);
            pythonClasses.pr(PythonReactionGenerator.generatePythonReactions(reactor, reactionToGenerate));
            pythonClasses.unindent();
            pythonClasses.pr("\n");
            instantiatedClasses.add(className);
        }

        for (ReactorInstance child : instance.children) {
            pythonClasses.pr(generatePythonClass(child, instantiatedClasses, main, types));
        }
        return pythonClasses.getCode();
    }

    private static String generatePythonClassHeader(String className) {
        return String.join("\n",
            "# Python class for reactor "+className+"",
            "class _"+className+":"
        );
    }

    /**
     * Generate code that instantiates and initializes parameters and state variables for a reactor 'decl'.
     *
     * @param decl The reactor declaration
     * @return The generated code as a StringBuilder
     */
     private static String generatePythonConstructor(ReactorDecl decl, PythonTypes types) {
        CodeBuilder code = new CodeBuilder();
        code.pr("# Constructor");
        code.pr("def __init__(self, **kwargs):");
        code.indent();
        code.pr(PythonParameterGenerator.generatePythonInstantiations(decl, types));
        code.pr(PythonStateGenerator.generatePythonInstantiations(decl));
        return code.toString();
    }

    /**
     * Generate code to instantiate a Python list that will hold the Python
     * class instance of reactor <code>instance<code>. Will recursively do
     * the same for the children of <code>instance<code> as well.
     *
     * @param instance The reactor instance for which the Python list will be created.
     */
    public static String generateListsToHoldClassInstances(ReactorInstance instance) {
        CodeBuilder code = new CodeBuilder();
        code.pr(PyUtil.reactorRefName(instance)+" = [None] * "+instance.getTotalWidth());
        for (ReactorInstance child : instance.children) {
            code.pr(generateListsToHoldClassInstances(child));
        }
        return code.toString();
    }

    /**
     * Instantiate classes in Python, as well as subclasses.
     * Instances are always instantiated as a list of className = [_className, _className, ...] depending on the size of the bank.
     * If there is no bank or the size is 1, the instance would be generated as className = [_className]
     * @param instance The reactor instance to be instantiated
     * @param main The main reactor
     */
    public static String generatePythonClassInstantiations(ReactorInstance instance,
                        ReactorInstance main) {
        CodeBuilder code = new CodeBuilder();

        String className = PyUtil.getName(instance.reactorDeclaration);

        if (instance.getWidth() > 0) {
            // For each reactor instance, create a list regardless of whether it is a bank or not.
            // Non-bank reactor instances will be a list of size 1.         var reactorClass = instance.definition.reactorClass
            String fullName = instance.getFullName();
            code.pr(String.join("\n",
                "# Start initializing "+fullName+" of class "+className,
                "for "+PyUtil.bankIndexName(instance)+" in range("+instance.getWidth()+"):"
            ));
            code.indent();
            // Define a bank_index local variable so that it can be used while
            // setting parameter values.
            code.pr("bank_index = "+PyUtil.bankIndexName(instance));
            code.pr(generatePythonClassInstantiation(instance, className));
        }

        for (ReactorInstance child : instance.children) {
            code.pr(generatePythonClassInstantiations(child, main));
        }
        code.unindent();
        return code.toString();
    }

    /**
     * Instantiate a class with className in instance.
     * @param instance The reactor instance to be instantiated
     * @param className The name of the class to instantiate
     */
    private static String generatePythonClassInstantiation(ReactorInstance instance,
                                                           String className) {
        CodeBuilder code = new CodeBuilder();
        code.pr(PyUtil.reactorRef(instance)+" = _"+className+"(");
        code.indent();
        // Always add the bank_index
        code.pr("_bank_index = "+PyUtil.bankIndex(instance)+",");
        for (ParameterInstance param : instance.parameters) {
            if (!param.getName().equals("bank_index")) {
                code.pr("_"+param.getName()+"="+ PythonParameterGenerator.generatePythonInitializer(param)+",");
            }
        }
        code.unindent();
        code.pr(")");
        return code.toString();
    }
}
