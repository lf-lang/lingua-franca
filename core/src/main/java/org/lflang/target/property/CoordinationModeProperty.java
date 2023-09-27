package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;

import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.target.property.CoordinationModeProperty.CoordinationMode;
import org.lflang.target.property.type.UnionType;
import org.lflang.validation.ValidationReporter;

public class CoordinationModeProperty extends TargetPropertyConfig<CoordinationMode> {

    public CoordinationModeProperty() {
        super(UnionType.COORDINATION_UNION);
    }

    @Override
    public CoordinationMode initialize() {
        return CoordinationMode.CENTRALIZED;
    }

    @Override
    public CoordinationMode parse(Element value) {
        return (CoordinationMode) UnionType.COORDINATION_UNION.forName(ASTUtils.elementToSingleString(value));
    }

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.C, Target.CCPP, Target.Python);
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
