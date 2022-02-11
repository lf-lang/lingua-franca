package org.lflang.generator.python;

import org.lflang.lf.ReactorDecl;
import org.lflang.generator.c.CUtil;

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
}
