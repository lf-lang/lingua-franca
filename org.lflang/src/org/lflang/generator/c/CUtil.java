/* Utilities for C code generation. */

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

package org.lflang.generator.c;

import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;

/**
 * A collection of utilties for C code generation.
 * This class codifies the coding conventions for the C target code generator.
 * I.e., it defines how variables are named and referenced.
 */
public class CUtil {

    //////////////////////////////////////////////////////
    //// Public methods.

    /**
     * Return a name of a variable to refer to the bank index of a reactor
     * in a bank. This is has the form INDEX_PREFIX_i where i is the depth
     * of the reactor (how many parents it has).
     * @param instance A reactor instance.
     */
    static public String bankIndex(ReactorInstance instance) {
        return INDEX_PREFIX + instance.getDepth();
    }
    
    /**
     * Return a string for referencing the port struct (which has the value
     * and is_present fields) in the self struct of the port's parent's
     * parent.  This is used by reactions that are triggered by an output
     * of a contained reactor and by reactions that send data to inputs
     * of a contained reactor. This will have one of the following forms:
     * 
     * * selfStructRef->_lf_reactorName.portName
     * * selfStructRef->_lf_reactorName[id].portName
     * 
     * Where the selfStructRef is as returned by selfRef().
     * 
     * @param port The port.
     */
    static public String containedPortRef(PortInstance port) {
        var destStruct = CUtil.selfRef(port.getParent().getParent());
        return destStruct + "->_lf_" + reactorRef(port.getParent()) + "." + port.getName();
    }

    /**
     * Return a string for referencing the struct with the value and is_present
     * fields of the specified port. This is used for establishing the destination of
     * data for a connection between ports.
     * This will have the following form:
     * 
     * * selfStruct->_lf_portName
     * 
     * @param port An instance of a destination input port.
     */
    static public String destinationRef(PortInstance port) {
        // Note that if the port is an output, then it must
        // have dependent reactions, otherwise it would not
        // be a destination.
        return selfRef(port.getParent()) + "->_lf_" + port.getName();
    }

    /** 
     * Return a reference to the specified reactor instance within a self
     * struct. The result has one of the following forms:
     * 
     * * instanceName
     * * instanceName[id]
     * 
     * where "id" is a variable referring to the bank index if the reactor
     * is a bank.
     * 
     * @param instance The reactor instance.
     * @return A reference to the reactor within a self struct.
     */
    static public String reactorRef(ReactorInstance instance) {
        // If this reactor is a member of a bank of reactors, then change
        // the name of its self struct to append [index].
        if (instance.isBank()) {
            return instance.getName() + "[" + INDEX_PREFIX + instance.getDepth() + "]";
        } else {
            return instance.getName();
        }
    }

    /** 
     * Return the unique reference for the "self" struct of the specified
     * reactor instance. If the instance is a bank of reactors, this returns
     * something of the form name_self[id], where i is INDEX_PREFIX and 
     * d is the depth of the reactor. This assumes that the resulting string
     * will be used in a context that defines a variable id. The result has
     * one of the following forms:
     * 
     * * uniqueID
     * * uniqueID[id]
     * 
     * @param instance The reactor instance.
     * @return A reference to the self struct.
     */
    static public String selfRef(ReactorInstance instance) {
        var result = instance.uniqueID() + "_self";
        // If this reactor is a member of a bank of reactors, then change
        // the name of its self struct to append [index].
        if (instance.isBank()) {
            result += "[" + INDEX_PREFIX + instance.getDepth() + "]";
        }
        return result;
    }

    //////////////////////////////////////////////////////
    //// Public constants.

    /** Prefix used for-loop variables when iterating over bank members. */
    public static String INDEX_PREFIX = "i_";
}
