/* Base class for instances with names in Lingua Franca. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

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

package org.icyphy.generator

import java.util.HashMap
import org.eclipse.emf.ecore.EObject

/** Base class for instances with names in Lingua Franca.
 *  
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
abstract class NamedInstance<T extends EObject> {
        
    /** Construct a new instance with the specified definition
     *  and parent. E.g., for a reactor instance, the definition
     *  is Instantiation, and for a port instance, it is Port. These are
     *  nodes in the AST. This is protected because only subclasses
     *  should be constructed.
     *  @param definition The definition in the AST for this instance.
     *  @param parent The reactor instance that creates this instance.
     */
    protected new(T definition, ReactorInstance parent) {
        this.definition = definition;
        this.parent = parent
    }
    
    //////////////////////////////////////////////////////
    //// Public fields.
    
    /** The Instantiation AST object from which this was created. */
    public var T definition
    
    /** A limit on the number of characters returned by uniqueID. */
    public static var identifierLengthLimit = 40
    
    /** The reactor instance that creates this instance. */
    public var ReactorInstance parent

    //////////////////////////////////////////////////////
    //// Public methods.

    /** Return the full name of this instance, which has the form
     *  "a.b.c", where "c" is the name of this instance, "b" is the name
     *  of its container, and "a" is the name of its container, stopping
     *  at the container in main.
     *  @return The full name of this instance.
     */
    def String getFullName() {
        getFullNameWithJoiner('.')
    }
    
    /** Return the name of this instance as given in its definition.
     *  Note that this is unique only relative to other instances with
     *  the same parent.
     *  @return The name of this instance within its parent.
     */
    abstract def String getName();
    
    /** Return the main reactor, which is the top-level parent.
     *  @return The top-level parent.
     */
    abstract def ReactorInstance main();

    /** Return an identifier for this instance, which has the form "a_b_c"
     *  or "a_b_c_n", where "c" is the name of this instance, "b" is the name
     *  of its container, and "a" is the name of its container, stopping
     *  at the container in main. All names are converted to lower case.
     *  The suffix _n is usually omitted, but it is possible to get name
     *  collisions using the above scheme, in which case _n will be an
     *  increasing integer until there is no collision.
     *  If the length of the root of the name as calculated above (the root
     *  is without the _n suffix) is longer than
     *  the static variable identifierLengthLimit, then the name will be
     *  truncated. The returned name will be the tail of the name calculated
     *  above with the prefix '__'.
     *  @return An identifier for this instance that is guaranteed to be
     *   unique within the top-level parent.
     */
    def String uniqueID() {
        if (_uniqueID === null) {
            // Construct the unique ID only if it has not been
            // previously constructed.
            var prefix = getFullNameWithJoiner('_').toLowerCase
            
            // Truncate, if necessary.
            if (prefix.length > identifierLengthLimit) {
                prefix = '__' 
                    + prefix.substring(prefix.length - identifierLengthLimit + 2)
            }
            
            // Ensure uniqueness.
            var toplevel = main()
            if (toplevel._uniqueIDCount === null) {
                toplevel._uniqueIDCount = new HashMap<String,Integer>()
            }
            var count = toplevel._uniqueIDCount.get(prefix)
            if (count === null) {
                toplevel._uniqueIDCount.put(prefix, 1)
                _uniqueID = prefix
            } else {
                toplevel._uniqueIDCount.put(prefix, count + 1)
                // NOTE: The length of this name could exceed
                // identifierLengthLimit. Is this OK?
                _uniqueID = prefix + '_' + (count + 1)
            }
        }
        _uniqueID
    }

    //////////////////////////////////////////////////////
    //// Protected fields.

    /** Map from a name of the form a_b_c to the number of
     *  unique IDs with that prefix that have been already
     *  assigned. If none have been assigned, then there is
     *  no entry in this map. This map should be non-null only
     *  for the main reactor (the top level).
     */
    protected HashMap<String,Integer> _uniqueIDCount;

    //////////////////////////////////////////////////////
    //// Protected methods.

    /** Return a string of the form
     *  "a.b.c", where "." is replaced by the specified joiner,
     *  "c" is the name of this instance, "b" is the name
     *  of its container, and "a" is the name of its container, stopping
     *  at the container in main.
     *  @return A string representing this instance.
     */
    protected def String getFullNameWithJoiner(String joiner) {
        if (this.parent === null) {
            this.getName
        } else {
            parent.getFullNameWithJoiner(joiner) + joiner + this.getName
        }
    }

    //////////////////////////////////////////////////////
    //// Private fields.
    
    /** Unique ID for this instance. This is null until
     *  uniqueID() is called.
     */
    String _uniqueID = null;
}