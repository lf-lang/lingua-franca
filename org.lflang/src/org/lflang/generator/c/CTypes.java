/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.c;

import java.util.stream.Collectors;

import org.lflang.InferredType;
import org.lflang.generator.TargetTypes;
import org.lflang.generator.UnsupportedGeneratorFeatureException;
import org.lflang.lf.Initializer;
import org.lflang.lf.TimeUnit;

/**
 * {@link TargetTypes} impl for {@link CGenerator}.
 *
 * @author Clément Fournier
 */
public class CTypes implements TargetTypes {

    public static final CTypes INSTANCE = new CTypes();

    @Override
    public boolean supportsGenerics() {
        return false;
    }

    @Override
    public String getTargetTimeType() {
        return "interval_t";
    }

    @Override
    public String getTargetTagType() {
        return "tag_t";
    }

    @Override
    public String getTargetFixedSizeListType(String baseType, int size) {
        return baseType + "[" + size + "]";
    }

    @Override
    public String getTargetVariableSizeListType(String baseType) {
        return baseType + "[]";
    }

    @Override
    public String getTargetUndefinedType() {
        // todo C used to insert a marker in the code
        throw new UnsupportedGeneratorFeatureException("Undefined type");
    }

    @Override
    public String getTargetTimeExpr(long magnitude, TimeUnit unit) {
        if (unit != TimeUnit.NONE) {
            return unit.name() + '(' + magnitude + ')';
        } else {
            return Long.toString(magnitude);
        }
    }

    @Override
    public String getTargetInitializerWithNotExactlyOneValue(Initializer init, InferredType type) {
        return init.getExprs().stream()
                   .map(it -> getTargetExpr(it, type.getComponentType()))
                   .collect(Collectors.joining(", ", "{", "}"));
    }
}
