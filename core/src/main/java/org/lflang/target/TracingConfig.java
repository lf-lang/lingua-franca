package org.lflang.target;

import java.util.Objects;

import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.TargetPropertyConfig;
import org.lflang.target.TracingConfig.TracingOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.validation.ValidationReporter;

public class TracingConfig extends TargetPropertyConfig<TracingOptions> {


    @Override
    public TracingOptions initialize() {
        return new TracingOptions();
    }

    @Override
    public TracingOptions parse(Element value) {
        var options = new TracingOptions();
        if (value.getLiteral() != null) {
            if (!ASTUtils.toBoolean(value)) {
                options.enabled = false;
            }
        } else {
            for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                TracingOption option =
                    (TracingOption) DictionaryType.TRACING_DICT.forName(entry.getName());
                switch (option) {
                case TRACE_FILE_NAME:
                    options.traceFileName = ASTUtils.elementToSingleString(entry.getValue());
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
        if (pair != null && this.parse(pair.getValue()) != null) {
            // If tracing is anything but "false" and threading is off, error.
            var threading = TargetProperty.getKeyValuePair(ast, TargetProperty.THREADING);
            if (threading != null) {
                if (!ASTUtils.toBoolean(threading.getValue())) {
                    reporter.error(
                        "Cannot enable tracing because threading support is disabled",
                        pair,
                        Literals.KEY_VALUE_PAIR__NAME);
                    reporter.error(
                        "Cannot disable treading support because tracing is enabled",
                        threading,
                        Literals.KEY_VALUE_PAIR__NAME);
                }
            }
        }
    }

    @Override
    public Element export() {
        if (this.value.isEnabled()) {
            return null;
        } else if (this.value.equals(new TracingOptions())) {
            // default values
            return ASTUtils.toElement(true);
        } else {
            Element e = LfFactory.eINSTANCE.createElement();
            KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
            for (TracingOption opt : TracingOption.values()) {
                KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
                pair.setName(opt.toString());
                switch (opt) {
                case TRACE_FILE_NAME:
                    if (this.value.traceFileName == null) {
                        continue;
                    }
                    pair.setValue(ASTUtils.toElement(this.value.traceFileName));
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

    /** Settings related to tracing options. */
    public static class TracingOptions {

        protected boolean enabled = true;

        /**
         * The name to use as the root of the trace file produced. This defaults to the name of the .lf
         * file.
         */
        public String traceFileName = null;

        @Override
        public boolean equals(Object o) {
            if (this == o) {
                return true;
            }
            if (o == null || getClass() != o.getClass()) {
                return false;
            }
            TracingOptions that = (TracingOptions) o;
            return Objects.equals(traceFileName, that.traceFileName); // traceFileName may be null
        }

        public boolean isEnabled() {
            return enabled;
        }
    }


    /**
     * Tracing options.
     *
     * @author Edward A. Lee
     */
    public enum TracingOption implements DictionaryElement {
        TRACE_FILE_NAME("trace-file-name", PrimitiveType.STRING);

        public final PrimitiveType type;

        private final String description;

        private TracingOption(String alias, PrimitiveType type) {
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
