package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.TargetPropertyConfig;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.target.CoordinationConfig.CoordinationOption;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.validation.LFValidator.ValidationReporter;

public class CoordinationConfig implements TargetPropertyConfig<CoordinationOption> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {

    }

    @Override
    public CoordinationOption parse(Element value) {
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
     * Coordination options.
     *
     * @author Edward A. Lee
     */
    public enum CoordinationOption implements DictionaryElement {
        ADVANCE_MESSAGE_INTERVAL("advance-message-interval", PrimitiveType.TIME_VALUE);

        public final PrimitiveType type;

        private final String description;

        private CoordinationOption(String alias, PrimitiveType type) {
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

    /**
     * Enumeration of coordination types.
     *
     * @author Marten Lohstroh
     */
    public enum CoordinationType {
        CENTRALIZED,
        DECENTRALIZED;

        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }


}
