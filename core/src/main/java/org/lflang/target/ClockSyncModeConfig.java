package org.lflang.target;


import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.target.ClockSyncModeConfig.ClockSyncMode;
import org.lflang.target.property.type.UnionType;
import org.lflang.validation.ValidationReporter;

public class ClockSyncModeConfig extends TargetPropertyConfig<ClockSyncMode> {


    @Override
    public ClockSyncMode initialize() {
        return ClockSyncMode.INIT;
    }

    @Override
    public ClockSyncMode parse(Element value) {

        UnionType.CLOCK_SYNC_UNION.validate(value);
        var mode = (ClockSyncMode)
            UnionType.CLOCK_SYNC_UNION.forName(ASTUtils.elementToSingleString(value));
        if (mode != null) {
            return mode;
        } else {
            return ClockSyncMode.INIT;
        }
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
    public Element export() {
        return ASTUtils.toElement(this.value.toString());
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
        ON;
        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }
}
