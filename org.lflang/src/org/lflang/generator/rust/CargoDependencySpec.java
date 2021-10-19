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

package org.lflang.generator.rust;

import java.util.LinkedHashMap;
import java.util.Map;

import org.lflang.TargetProperty;
import org.lflang.TargetProperty.TargetPropertyType;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage;
import org.lflang.validation.LFValidator;

/**
 * Info about a cargo dependency. See {@link TargetProperty#CARGO_DEPENDENCIES}.
 *
 * @author Clément Fournier
 */
public class CargoDependencySpec {

    final String version;
    final String localPath;

    private CargoDependencySpec(String version, String localPath) {
        this.version = version;
        this.localPath = localPath;
    }

    /** The version, may have surrounding quotes. */
    public String getVersion() {
        return version;
    }

    /** Local path to the crate, may have surrounding quotes. */
    public String getLocalPath() {
        return localPath;
    }

    public static Map<String, CargoDependencySpec> parseAll(Element element) {
        var result = new LinkedHashMap<String, CargoDependencySpec>();
        for (KeyValuePair pair : element.getKeyvalue().getPairs()) {
            result.put(pair.getName(), parseValue(pair.getValue()));
        }
        return result;
    }

    private static CargoDependencySpec parseValue(Element element) {
        if (element.getLiteral() != null) {
            return new CargoDependencySpec(element.getLiteral(), null);
        } else if (element.getKeyvalue() != null) {
            String version = null;
            String localPath = null;
            for (KeyValuePair pair : element.getKeyvalue().getPairs()) {
                String name = pair.getName();
                String literal = pair.getValue().getLiteral();
                if (literal == null) {
                    throw new InvalidLfSourceException(pair.getValue(), "Expected string literal for key '" + name + "'");
                }
                if (name.equals("version")) {
                    version = literal;
                } else if (name.equals("path")) {
                    localPath = literal;
                } else {
                    throw new InvalidLfSourceException(pair, "Unknown key: '" + name + "'");
                }
            }
            if (version != null || localPath != null) {
                return new CargoDependencySpec(version, localPath);
            } else {
                throw new InvalidLfSourceException(element.getKeyvalue(), "Must specify one of 'version' or 'path'");
            }
        }
        throw new InvalidLfSourceException(element, "Expected string or dictionary");
    }

    /**
     * The property type for the
     */
    public static final class CargoDependenciesPropertyType implements TargetPropertyType {

        public static final TargetPropertyType INSTANCE = new CargoDependenciesPropertyType();

        private CargoDependenciesPropertyType() {
        }

        @Override
        public boolean validate(Element e) {
            return e.getKeyvalue() != null;
        }

        @Override
        public void check(Element element, String name, LFValidator v) {
            for (KeyValuePair pair : element.getKeyvalue().getPairs()) {
                try {
                    parseValue(pair.getValue());
                } catch (InvalidLfSourceException e) {
                    v.getErrorReporter().reportError(e.getNode(), e.getProblem());
                }
            }
        }

        @Override
        public String toString() {
            return "<cargo dependency spec>";
        }
    }
}
