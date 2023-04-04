/* Utilities for Python code generation. */

/*************
Copyright (c) 2019-2021, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.generator.python;

import org.lflang.generator.ReactorInstance;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Expression;
import org.lflang.lf.ParameterReference;
import org.lflang.ASTUtils;


/**
 * A collection of utilities for Python code generation.
 * This class inherits from CUtil but overrides a few methods to
 * codify the coding conventions for the Python target code generator.
 * I.e., it defines how some variables are named and referenced.
 * @author Edward A. Lee
 * @author Soroush Bateni
 */
public class PyUtil extends CUtil {

    /**
     * Return the name of the list of Python class instances that contains the
     * specified reactor instance. This is similar to
     * {@link #reactorRef(ReactorInstance)} except that it does not index into
     * the list.
     *
     * @param instance The reactor instance.
     */
    public static String reactorRefName(ReactorInstance instance) {
        return instance.uniqueID() + "_lf";
    }

    /**
     * Return a reference to the list of Python class instances that contains
     * the specified reactor instance. The returned string has the form
     * list_name[runtimeIndex], where list_name is the name of the list of
     * Python class instances that contains this reactor instance. If
     * runtimeIndex is null, then it is replaced by the expression returned by
     * {@link runtimeIndex(ReactorInstance)} or 0 if there are no banks.
     *
     * @param instance     The reactor instance.
     * @param runtimeIndex An optional expression to use to address bank
     *                     members. If this is null, the expression used will be
     *                     that returned by
     *                     {@link #runtimeIndex(ReactorInstance)}.
     */
    public static String reactorRef(ReactorInstance instance, String runtimeIndex) {
        if (runtimeIndex == null) runtimeIndex = runtimeIndex(instance);
        return PyUtil.reactorRefName(instance) + "[" + runtimeIndex + "]";
    }

    /**
     * Return a reference to the list of Python class instances that contains
     * the specified reactor instance. The returned string has the form
     * list_name[j], where list_name is the name of the list of of Python class
     * instances that contains this reactor instance and j is the expression
     * returned by {@link #runtimeIndex(ReactorInstance)} or 0 if there are no
     * banks.
     *
     * @param instance The reactor instance.
     */
    public static String reactorRef(ReactorInstance instance) {
        return PyUtil.reactorRef(instance, null);
    }

    /**
     * Convert C types to formats used in Py_BuildValue and PyArg_PurseTuple.
     * This is unused but will be useful to enable inter-compatibility between
     * C and Python reactors.
     * @param type C type
     */
    public static String pyBuildValueArgumentType(String type) {
        switch (type) {
            case "int":                return "i";
            case "string":             return "s";
            case "char":               return "b";
            case "short int":          return "h";
            case "long":               return "l";
            case "unsigned char":      return "B";
            case "unsigned short int": return "H";
            case "unsigned int":       return "I";
            case "unsigned long":      return "k";
            case "long long":          return "L";
            case "interval_t":         return "L";
            case "unsigned long long": return "K";
            case "double":             return "d";
            case "float":              return "f";
            case "Py_complex":         return "D";
            case "Py_complex*":        return "D";
            case "Py_Object":          return "O";
            case "Py_Object*":         return "O";
            default:                   return "O";
        }
    }

    public static String generateGILAcquireCode() {
        return String.join("\n",
            "// Acquire the GIL (Global Interpreter Lock) to be able to call Python APIs.",
            "PyGILState_STATE gstate;",
            "gstate = PyGILState_Ensure();"
        );
    }

    public static String generateGILReleaseCode() {
        return String.join("\n",
            "/* Release the thread. No Python API allowed beyond this point. */",
            "PyGILState_Release(gstate);"
        );
    }

    /**
     * Override to convert some C types to their
     * Python equivalent.
     * Examples:
     * true/false -> True/False
     * @param expr A value
     * @return A value string in the target language
     */
    protected static String getPythonTargetValue(Expression expr) {
        String returnValue;
        switch (ASTUtils.toOriginalText(expr)) {
            case "false":
                returnValue = "False";
                break;
            case "true":
                returnValue = "True";
                break;
            default: 
                returnValue = GeneratorBase.getTargetValue(expr);
        }

        // Parameters in Python are always prepended with a 'self.'
        // predicate. Therefore, we need to append the returned value
        // if it is a parameter.
        if (expr instanceof ParameterReference) {
            returnValue = "self." + returnValue;
        }

        return returnValue;
    }
}
