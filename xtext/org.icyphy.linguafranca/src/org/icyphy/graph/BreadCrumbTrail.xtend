package org.icyphy.graph

/**
 * Class that serves to identify paths through the AST.
 * 
 * It assumed that the supplied identifiers uniquely identify each crumb when
 * combined with the identifiers that preceded it in a depth-first traversal
 * starting from the root of the tree. 
 */
class BreadCrumbTrail<T> {
    
    /**
     * An identifier built incrementally each time a new crumb is appended.
     */
    String trail
    
    /**
     * The last crumb that was appended. Together with the trail it identifies
     * a path through the AST.
     */
    T crumb
    
    /**
     * The next identifier to be appended to the trail.
     */
    String id
    
    /**
     * Create a new bread crumb trail.
     * 
     * @param trail A string prefix necessary to identify the crumb.
     * @param crumb The current crumb.
     * @param id Identifier associated with the current crumb. It will be
     * joined with trail string upon the next invocation of
     * {@link #BreadCrumbTrail.append append}.
     */    
    new(String trail, T crumb, String id) {
        this.trail = trail
        this.crumb = crumb
        this.id = id
    }
    
    /**
     * Append a new crumb to the trail.
     * 
     * If the crumb is null, just return the current object, otherwise return
     * a new bread crumb trail that has as a prefix the current trail.
     * @param The crumb to the added to the trail.
     * @param A string that identifies the crumb in combination with the trail
     * that has been constructed so far.
     */
    def BreadCrumbTrail<T> append(T crumb, String id) {
        if (crumb === null) {
            return this
        } else {
            return new BreadCrumbTrail(this.toString, crumb, id)
        }
    }
    
    /**
     * Report whether or not this object is equal to the given argument.
     * 
     * Two trails are equal if their string representations are equal.
     * @param obj The object to compare against.
     */
    override equals(Object obj) {
        if (obj instanceof BreadCrumbTrail) {
            return this.toString.equals(obj.toString)
        }
        return false
    }
    
    /**
     * Return the current trail which uniquely identifies the path from the
     * current crumb to the root of the tree. 
     */
    def getTrail() {
        return this.trail
    }
    
    /**
     * Return the current crumb.
     */
    def getCrumb() {
        return this.crumb
    }
    
    
    /**
     * Return a has code based on the string representation of this object.
     */
    override hashCode() {
        return this.toString.hashCode;
    }
    
    /**
     * Return the bread crumb trail including the trailing identifier
     * corresponding to the last crumb.
     */    
    override toString() {
        return '''«IF trail !== null && !trail.isEmpty»«this.trail».«ENDIF»«this.id»'''
    }
}
