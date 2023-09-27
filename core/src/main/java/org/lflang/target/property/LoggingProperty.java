package org.lflang.target.property;

import java.util.List;

import org.lflang.MessageReporter;
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
    protected LogLevel fromAst(Element value, MessageReporter err) {
        return fromString(ASTUtils.elementToSingleString(value), err);
    }

    protected LogLevel fromString(String string, MessageReporter err) {
        return LogLevel.valueOf(string.toUpperCase());
    }

    @Override
    public List<Target> supportedTargets() {
        return Target.ALL;
    }

    @Override
    public Element toAstElement() {
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
