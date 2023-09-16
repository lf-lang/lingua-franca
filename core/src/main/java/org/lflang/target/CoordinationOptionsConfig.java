package org.lflang.target;

import org.lflang.TargetConfig;

import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.TargetPropertyConfig;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.target.CoordinationOptionsConfig.CoordinationOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.validation.ValidationReporter;

public class CoordinationOptionsConfig extends TargetPropertyConfig<CoordinationOptions> {

    @Override
    public CoordinationOptions initialize() {
        return new CoordinationOptions();
    }

    @Override
    public CoordinationOptions parse(Element value) {
        var options = new CoordinationOptions();
        for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
            CoordinationOption option =
                (CoordinationOption) DictionaryType.COORDINATION_OPTION_DICT.forName(entry.getName());
            switch (option) {
            case ADVANCE_MESSAGE_INTERVAL:
                options.advanceMessageInterval =
                    ASTUtils.toTimeValue(entry.getValue());
                break;
            default:
                break;
            }
        }
        return options;
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        // FIXME
    }

    @Override
    public Element export() {
        Element e = LfFactory.eINSTANCE.createElement();
        KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
        for (CoordinationOption opt : CoordinationOption.values()) {
            KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
            pair.setName(opt.toString());
            switch (opt) {
            case ADVANCE_MESSAGE_INTERVAL:
                if (this.value.advanceMessageInterval == null) {
                    continue;
                }
                pair.setValue(
                    ASTUtils.toElement(value.advanceMessageInterval));
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

    /** Settings related to coordination of federated execution. */
    public static class CoordinationOptions {

        /**
         * For centralized coordination, if a federate has a physical action that can trigger an output,
         * directly or indirectly, then it will send NET (next event tag) messages to the RTI
         * periodically as its physical clock advances. This option sets the amount of time to wait
         * between sending such messages. Increasing this value results in downstream federates that lag
         * further behind physical time (if the "after" delays are insufficient). The default is null,
         * which means it is up the implementation to choose an interval.
         */
        public TimeValue advanceMessageInterval = null;
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

}
