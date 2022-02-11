package org.lflang.generator.python;

import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Instantiation;
import org.lflang.generator.c.CUtil;
import org.lflang.generator.c.CGenerator;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.CodeBuilder;
import org.lflang.ASTUtils;

public class PythonReactionGenerator {
    public static String generateCallPythonReactionCode(ReactorDecl decl, 
                                                        int reactionIndex, 
                                                        StringBuilder pyObjectDescriptor,
                                                        StringBuilder pyObjects) {
        String pythonFunctionName = PyUtil.generatePythonReactionFunctionName(reactionIndex);
        return String.join("\n", 
            "DEBUG_PRINT(\"Calling reaction function "+decl.getName()+"."+pythonFunctionName+"\");",
            "PyObject *rValue = PyObject_CallObject(",
            "    self->_lf_py_reaction_function_"+reactionIndex+", ",
            "    Py_BuildValue(\"("+pyObjectDescriptor+")\" "+pyObjects+")",
            ");",
            "if (rValue == NULL) {",
            "    error_print(\"FATAL: Calling reaction "+decl.getName()+"."+pythonFunctionName+" failed.\");",
            "    if (PyErr_Occurred()) {",
            "        PyErr_PrintEx(0);",
            "        PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "    }",
            "    "+PyUtil.generateGILReleaseCode(),
            "    Py_FinalizeEx();",
            "    exit(1);",
            "}",
            "",
            "/* Release the thread. No Python API allowed beyond this point. */",
            PyUtil.generateGILReleaseCode()
        );
    }

    public static String generateDeadlineViolationCode(ReactorDecl decl,
                                                       int reactionIndex, 
                                                       StringBuilder pyObjectDescriptor,
                                                       StringBuilder pyObjects) {
        String deadlineFunctionName = CUtil.generateDeadlineFunctionName(decl, reactionIndex);
        return String.join("\n", 
            ""+PyUtil.generateGILAcquireCode()+"",
            "",
            "DEBUG_PRINT(\"Calling deadline function "+decl.getName()+"."+deadlineFunctionName+"\");",
            "PyObject *rValue = PyObject_CallObject(",
            "    self->_lf_py_deadline_function_"+reactionIndex+", ",
            "    Py_BuildValue(\"("+pyObjectDescriptor+")\" "+pyObjects+")",
            ");",
            "if (rValue == NULL) {",
            "    error_print(\"FATAL: Calling reaction "+decl.getName()+"."+deadlineFunctionName+" failed. \\n);",
            "    if (rValue == NULL) {",
            "        if (PyErr_Occurred()) {",
            "            PyErr_PrintEx(0);",
            "            PyErr_Clear(); // this will reset the error indicator so we can run Python code again",
            "        }",
            "    }",
            "    /* Release the thread. No Python API allowed beyond this point. */",
            "    "+PyUtil.generateGILReleaseCode(),
            "    Py_FinalizeEx();",
            "    exit(1);",
            "}",
            "",
            "/* Release the thread. No Python API allowed beyond this point. */",
            PyUtil.generateGILReleaseCode()
        );
    }

    public static String generateDeadlineFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String deadlineFunctionName = CUtil.generateDeadlineFunctionName(decl, reactionIndex);
        return "void " + deadlineFunctionName + "(void* instance_args)";
    }

    public static String generateReactionFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String deadlineFunctionName = CUtil.generateReactionFunctionName(decl, reactionIndex);
        return "void " + deadlineFunctionName + "(void* instance_args)";
    }

    public static void _generateReaction(Reaction reaction, 
                                         ReactorDecl decl, 
                                         int reactionIndex, 
                                         CodeBuilder code, 
                                         Instantiation mainDef, 
                                         CGenerator g) {
        Reactor reactor = ASTUtils.toDefinition(decl);

        // Delay reactors and top-level reactions used in the top-level reactor(s) in federated execution are generated in C
        if (reactor.getName().contains(GeneratorBase.GEN_DELAY_CLASS_NAME) ||
            ((mainDef != null && decl == mainDef.getReactorClass() || mainDef == decl) && reactor.isFederated())) {
            g.generateReaction(reaction, decl, reactionIndex);
            return;
        }

        // Contains "O" characters. The number of these characters depend on the number of inputs to the reaction
        StringBuilder pyObjectDescriptor = new StringBuilder();

        // Contains the actual comma separated list of inputs to the reaction of type generic_port_instance_struct or generic_port_instance_with_token_struct.
        // Each input must be cast to (PyObject *)
        StringBuilder pyObjects = new StringBuilder();
        code.pr(generateReactionFunctionHeader(decl, reactionIndex) + " {");
        code.indent();

        // First, generate C initializations
        g.generateInitializationForReaction("", reaction, decl, reactionIndex);
        
        code.prSourceLineNumber(reaction.getCode());

        // Ensure that GIL is locked
        code.pr(PyUtil.generateGILAcquireCode());
    
        // Generate Python-related initializations
        generatePythonInitializationForReaction(reaction, decl, pyObjectDescriptor, pyObjects);
        
        // Call the Python reaction
        code.pr(generateCallPythonReactionCode(decl, reactionIndex, pyObjectDescriptor, pyObjects));
        
        code.unindent();
        code.pr("}");
        
        // Now generate code for the deadline violation function, if there is one.
        if (reaction.getDeadline() != null) {
            // The following name has to match the choice in generateReactionInstances
            code.pr(generateDeadlineFunctionHeader(decl, reactionIndex) + " {");
            code.indent();
            
            g.generateInitializationForReaction("", reaction, decl, reactionIndex);
        
            code.pr(generateDeadlineViolationCode(decl, reactionIndex, pyObjectDescriptor, pyObjects));
            
            code.unindent();
            code.pr("}");
        }
    }
}
