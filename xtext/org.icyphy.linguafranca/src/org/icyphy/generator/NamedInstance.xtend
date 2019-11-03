package org.icyphy.generator

import java.util.HashMap
import org.eclipse.emf.ecore.EObject

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
     *  @return An identifier for this instance that is guaranteed to be
     *   unique within the top-level parent.
     */
    def String uniqueID() {
        // FIXME: Limit the length.
        if (_uniqueID === null) {
            // Construct the unique ID only if it has not been
            // previously constructed.
            var prefix = getFullNameWithJoiner('_')
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