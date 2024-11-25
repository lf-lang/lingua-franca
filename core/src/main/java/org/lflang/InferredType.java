/*************
 * Copyright (c) 2020, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang;

import java.util.function.Function;
import org.eclipse.emf.ecore.EObject;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Type;

/**
 * A helper class that represents an inferred type.
 *
 * <p>This class helps to separate the rules of type inference from code generation for types. It is
 * particularly useful in cases when the type is not given directly in LF code, but is inferred from
 * the context. In this case it could happen that no ASTNode representing the type does not exist.
 * For instance when a parameter type is inferred from a time value. All in all, this class provides
 * a clean interface between type inference in ASTUtils and code generation.
 *
 * <p>ASTUtils provides functionality to create an inferred type from Lingua Franca variables
 * (getInferredType). This inferred type can than be translated to target code using the code
 * generators or be converted to a general textual representation using toText().
 *
 * @author Christian Menard
 */
public class InferredType {

  /** The AST node representing the inferred type if such a node exists. */
  public final Type astType;

  /** A flag indicating whether the inferred type has the base type time. */
  public final boolean isTime;

  /** Private constructor */
  private InferredType(Type astType, boolean isTime) {
    this.astType = astType;
    this.isTime = isTime;
  }

  /** Check if the inferred type is undefined. */
  public boolean isUndefined() {
    return astType == null && !isTime;
  }

  /**
   * Convert the inferred type to its textual representation as it would appear in LF code, with
   * CodeMap tags inserted.
   */
  public String toText() {
    return toTextHelper(ASTUtils::toText);
  }

  /**
   * Convert the inferred type to its textual representation as it would appear in LF code, without
   * CodeMap tags inserted.
   */
  public String toOriginalText() {
    return toTextHelper(ASTUtils::toOriginalText);
  }

  private String toTextHelper(Function<EObject, String> toText) {
    if (astType != null) {
      return toText.apply(astType);
    } else if (isTime) {
      return "time";
    }
    return "";
  }

  /**
   * Convert the inferred type to its textual representation while ignoring any list qualifiers or
   * type arguments.
   *
   * @return Textual representation of this inferred type without list qualifiers
   */
  public String baseType() {
    if (astType != null) {
      return ASTUtils.baseType(astType);
    } else if (isTime) {
      return "time";
    }
    return "";
  }

  /**
   * Create an inferred type from an AST node.
   *
   * @param type an AST node
   * @return A new inferred type representing the given AST node
   */
  public static InferredType fromAST(Type type) {
    if (type == null) {
      return undefined();
    }
    return new InferredType(type, type.isTime());
  }

  /** Create an undefined inferred type. */
  public static InferredType undefined() {
    return new InferredType(null, false);
  }

  /** Create an inferred type representing time. */
  public static InferredType time() {
    return new InferredType(null, true);
  }
}
