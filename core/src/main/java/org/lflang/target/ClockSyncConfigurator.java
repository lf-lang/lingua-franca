package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.target.ClockSyncConfigurator.ClockSyncMode;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.UnionType;
import org.lflang.validation.LFValidator.ValidationReporter;

public class ClockSyncConfigurator implements TargetPropertyConfig<ClockSyncMode> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {
        config.clockSync = this.parse(value);

    }

    @Override
    public ClockSyncMode parse(Element value) {
        return (ClockSyncMode)
            UnionType.CLOCK_SYNC_UNION.forName(ASTUtils.elementToSingleString(value));
    }


    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        if (pair != null) {
            boolean federatedExists = false;
            for (Reactor reactor : ast.getReactors()) {
                if (reactor.isFederated()) {
                    federatedExists = true;
                }
            }
            if (!federatedExists) {
                reporter.warning(
                    "The clock-sync target property is incompatible with non-federated programs.",
                    pair,
                    Literals.KEY_VALUE_PAIR__NAME);
            }
        }
    }

    @Override
    public Element getPropertyElement(TargetConfig config) {
        return ASTUtils.toElement(config.clockSync.toString());
    }

    /**
     * Clock synchronization options.
     *
     * @author Marten Lohstroh
     */
    public enum ClockSyncOption implements DictionaryElement {
        ATTENUATION("attenuation", PrimitiveType.NON_NEGATIVE_INTEGER),
        LOCAL_FEDERATES_ON("local-federates-on", PrimitiveType.BOOLEAN),
        PERIOD("period", PrimitiveType.TIME_VALUE),
        TEST_OFFSET("test-offset", PrimitiveType.TIME_VALUE),
        TRIALS("trials", PrimitiveType.NON_NEGATIVE_INTEGER),
        COLLECT_STATS("collect-stats", PrimitiveType.BOOLEAN);

        public final PrimitiveType type;

        private final String description;

        private ClockSyncOption(String alias, PrimitiveType type) {
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
     * Enumeration of clock synchronization modes.
     *
     * <ul>
     *   <li>OFF: The clock synchronization is universally off.
     *   <li>STARTUP: Clock synchronization occurs at startup only.
     *   <li>ON: Clock synchronization occurs at startup and at runtime.
     * </ul>
     *
     * @author Edward A. Lee
     */
    public enum ClockSyncMode {
        OFF,
        INIT,
        ON; // TODO Discuss initial in now a mode keyword (same as startup) and cannot be used as target
        // property value, thus changed it to init
        // FIXME I could not test if this change breaks anything

        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }
}
