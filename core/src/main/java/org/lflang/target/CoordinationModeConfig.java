package org.lflang.target;

import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.target.CoordinationModeConfig.CoordinationMode;
import org.lflang.target.property.type.UnionType;
import org.lflang.validation.ValidationReporter;

public class CoordinationModeConfig extends TargetPropertyConfig<CoordinationMode> {

    @Override
    public CoordinationMode initialize() {
        return CoordinationMode.CENTRALIZED;
    }

    @Override
    public CoordinationMode parse(Element value) {
        return (CoordinationMode) UnionType.COORDINATION_UNION.forName(ASTUtils.elementToSingleString(value));
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {}

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value.toString());
    }

    /**
     * Enumeration of coordination types.
     *
     * @author Marten Lohstroh
     */
    public enum CoordinationMode {
        CENTRALIZED,
        DECENTRALIZED;

        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }

}
