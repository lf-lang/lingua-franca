/* A helper class that represents an inferred type. */

/*************
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy

import org.icyphy.linguaFranca.Type

import static extension org.icyphy.ASTUtils.*

/**
 * A helper class that represents an inferred type.
 * 
 * This class helps to separate the rules of type inference from code generation 
 * for types. It is particularly useful in cases when the type is not given directly
 * in LF code, but is inferred from the context. In this case it could happen that
 * no ASTNode representing the type does not exist. For instance when a
 * parameter type is inferred from a time value. All in all, this class provides a
 * clean interface between type inference in ASTUtils and code generation. 
 * 
 * ASTUtils provides functionality to create an inferred type from
 * Lingua Franca variables (getInferredType). This inferred type can than be
 * translated to target code using the code generators or be converted to a general
 * textual representation using toText().
 *    
 * @author{Christian Menard <christian.menard@tu-dresden.de>}
 */
class InferredType {
    /** The AST node representing the inferred type if such a note exists.  */    
    public val Type astType
    /** A flag indicating whether the inferred type has the base type time. */
    public val Boolean isTime
    /** A flag indicating whether the inferred type is a list. */
    public val Boolean isList
    /** A flag indicating whether the inferred type is a list of variable size. */
    public val Boolean isVariableSizeList
    /** A flag indicating whether the inferred type is a list of fixed size. */
    public val Boolean isFixedSizeList
    /** The list size if the inferred type is a fixed size list. */
    public val Integer listSize
    
    /** Check if the inferred type is undefined. */
    def isUndefined() { return astType === null && !isTime; }

    /**
     * Convert the inferred type to its textual representation as it would appear in LF code.
     * 
     * @return Textual representation of this inferred type
     */
    def String toText() {
        if (astType !== null) {
            return astType.toText
        } else if (isTime) {
            if (isFixedSizeList) {
                return '''time[«listSize.toString»]'''
            } else if (isVariableSizeList) {
                return "time[]"
            } else {
                return "time"
            }
        }
        return ""
    }
    
    /**
     * Convert the inferred type to its textual representation while ignoring any list qualifiers.
     * 
     * @return Textual representation of this inferred type without list qualifiers
     */
    def baseType() {
        if (astType !== null) {
            return astType.baseType
        } else if (isTime) {
            return "time"
        }
        return ""
    }
    
    /** Private constructor */
    private new(Type astType, Boolean isTime, Boolean isList, Boolean isVariableSizeList, 
            Boolean isFixedSizeList, Integer listSize) {
        this.astType = astType
        this.isTime = isTime 
        this.isList = isList
        this.isVariableSizeList = isVariableSizeList
        this.isFixedSizeList = isFixedSizeList
        this.listSize = listSize
    }
    
    /**
     * Create an inferred type from an AST node.
     * 
     * @param type an AST node
     * @return A new inferred type representing the given AST node
     */
    static def fromAST(Type type) {
        return new InferredType(
            type,
            type.isTime,
            type.arraySpec !== null,
            type.arraySpec !== null && type.arraySpec.isOfVariableLength,
            type.arraySpec !== null && !type.arraySpec.isOfVariableLength,
            type.arraySpec !== null ? type.arraySpec.length : null)
    }
    
    /**
     * Create an undefined inferred type.
     * 
     * @return A new undefined inferred type
     */
    static def undefined() {
        return new InferredType(null, false, false, false, false, null)
    }
    
    /**
     * Create an inferred type representing time.
     * 
     * @return A new inferred typerepresenting time
     */
    static def time() {
        return new InferredType(null, true, false, false, false, null)
    }
    
    /**
     * Create an inferred type representing a time list.
     * 
     * This creates a fixed size list if size is given and a variable size list
     * if size is null.
     * 
     * @param size The list size
     * @return A new inferred type representing a time list
     */
    static def timeList(Integer size) {
        return new InferredType(null, true, true, size === null, size !== null, size)
    }
    
    /**
     * Create an inferred type representing a variable size time list.
     * 
     * @return A new inferred type representing a variable size time list
     */
    static def timeList() {
        return timeList(null)
    }
}