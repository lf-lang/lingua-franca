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
import org.lflang.generator.c.CUtil;


/**
 * A collection of utilities for Python code generation.
 * This class inherits from CUtil but overrides a few methods to 
 * codify the coding conventions for the Python target code generator.
 * I.e., it defines how some variables are named and referenced.
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Soroush Bateni <soroush@utdallas.edu>}
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
    static public String reactorRefName(ReactorInstance instance) {
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
    static public String reactorRef(ReactorInstance instance, String runtimeIndex) {
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
    static public String reactorRef(ReactorInstance instance) {
        return PyUtil.reactorRef(instance, null);
    }

}
