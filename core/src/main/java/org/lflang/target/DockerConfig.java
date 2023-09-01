package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.TargetPropertyConfig;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.target.DockerConfig.DockerOption;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.validation.LFValidator.ValidationReporter;

public class DockerConfig implements TargetPropertyConfig<DockerOption> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {

    }

    @Override
    public DockerOption parse(Element value) {
        return null;
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {

    }

    @Override
    public Element getPropertyElement(TargetConfig config) {
        return null;
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
