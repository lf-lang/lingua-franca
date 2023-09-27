package org.lflang.target.property;


import java.util.List;

import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.LoggingProperty.LogLevel;
import org.lflang.target.property.type.UnionType;

public class LoggingProperty extends TargetPropertyConfig<LogLevel> {

    public LoggingProperty() {
        super(UnionType.LOGGING_UNION);
    }

    @Override
    public LogLevel initialValue() {
        return LogLevel.INFO;
    }

    @Override
    protected LogLevel parse(Element value) {
        return (LogLevel) UnionType.LOGGING_UNION.forName(ASTUtils.elementToSingleString(value));
    }

    @Override
    public List<Target> supportedTargets() {
        return Target.ALL;
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(value.toString());
    }

    /**
     * Log levels in descending order of severity.
     *
     * @author Marten Lohstroh
     */
    public enum LogLevel {
        ERROR,
        WARN,
        INFO,
        LOG,
        DEBUG;

        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }

}
