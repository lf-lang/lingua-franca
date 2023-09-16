package org.lflang.target;


import java.util.Properties;

import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.target.DockerConfig.DockerOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.validation.ValidationReporter;


public class DockerConfig extends TargetPropertyConfig<DockerOptions> {

    @Override
    public DockerOptions initialize() {
        return new DockerOptions(false);
    }

    @Override
    public void update(Properties cliArgs) {
        var key = TargetProperty.DOCKER.toString();
        if (cliArgs.containsKey(key)) {
            var arg = cliArgs.getProperty(key);
            if (Boolean.parseBoolean(arg)) {
                this.value.enabled = true;
            } else {
                this.value.enabled = false;
            }
        }
    }

    @Override
    public DockerOptions parse(Element value) {
        var options = new DockerOptions(false);
        if (value.getLiteral() != null) {
            if (ASTUtils.toBoolean(value)) {
                options.enabled = true;
            }
        } else {
            for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                DockerOption option = (DockerOption) DictionaryType.DOCKER_DICT.forName(entry.getName());
                switch (option) {
                case FROM:
                    options.from = ASTUtils.elementToSingleString(entry.getValue());
                    break;
                default:
                    break;
                }
            }
        }
        return options;
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {

    }

    @Override
    public Element export() {
        if (!this.value.enabled) {
            return null;
        } else if (this.value.equals(new DockerOptions(true))) {
            // default configuration
            return ASTUtils.toElement(true);
        } else {
            Element e = LfFactory.eINSTANCE.createElement();
            KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
            for (DockerOption opt : DockerOption.values()) {
                KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
                pair.setName(opt.toString());
                switch (opt) {
                case FROM:
                    if (this.value.from == null) {
                        continue;
                    }
                    pair.setValue(ASTUtils.toElement(this.value.from));
                    break;
                }
                kvp.getPairs().add(pair);
            }
            e.setKeyvalue(kvp);
            if (kvp.getPairs().isEmpty()) {
                return null;
            }
            return e;
        }
    }

    /** Settings related to Docker options. */
    public static class DockerOptions {

        public boolean enabled;

        public DockerOptions(boolean enabled) {
            this.enabled = enabled;
        }

        /**
         * The base image and tag from which to build the Docker image. The default is "alpine:latest".
         */
        public String from = "alpine:latest";

        @Override
        public boolean equals(Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }
            DockerOptions that = (DockerOptions) o;
            return from.equals(that.from);
        }
    }



    /**
     * Docker options.
     *
     * @author Edward A. Lee
     */
    public enum DockerOption implements DictionaryElement {
        FROM("FROM", PrimitiveType.STRING);

        public final PrimitiveType type;

        private final String description;

        private DockerOption(String alias, PrimitiveType type) {
            this.description = alias;
            this.type = type;
        }

        /** Return the description of this dictionary element. */
        @Override
        public String toString() {
            return this.description;
        }

        /** Return the type associated with this dictionary element. */
        public TargetPropertyType getType() {
            return this.type;
        }
    }
}
