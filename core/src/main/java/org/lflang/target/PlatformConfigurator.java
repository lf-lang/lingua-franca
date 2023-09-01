package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.DictionaryElement;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.TargetPropertyConfig;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Model;
import org.lflang.target.PlatformConfigurator.PlatformOption;
import org.lflang.validation.LFValidator.ValidationReporter;

public class PlatformConfigurator implements TargetPropertyConfig<PlatformOption> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {

    }

    @Override
    public PlatformOption parse(Element value) {
        return null;
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        var threading = TargetProperty.getKeyValuePair(ast, TargetProperty.THREADING);
        if (threading != null) {
            if (pair != null && ASTUtils.toBoolean(threading.getValue())) {
                var lit = ASTUtils.elementToSingleString(pair.getValue());
                var dic = pair.getValue().getKeyvalue();
                if (lit != null && lit.equalsIgnoreCase(Platform.RP2040.toString())) {
                    reporter.error(
                        "Platform " + Platform.RP2040 + " does not support threading",
                        pair,
                        Literals.KEY_VALUE_PAIR__VALUE);
                }
                if (dic != null) {
                    var rp =
                        dic.getPairs().stream()
                            .filter(
                                kv ->
                                    kv.getName().equalsIgnoreCase("name")
                                        && ASTUtils.elementToSingleString(kv.getValue())
                                        .equalsIgnoreCase(Platform.RP2040.toString()))
                            .findFirst();
                    if (rp.isPresent()) {
                        reporter.error(
                            "Platform " + Platform.RP2040 + " does not support threading",
                            rp.get(),
                            Literals.KEY_VALUE_PAIR__VALUE);
                    }
                }
            }
        }
    }

    @Override
    public Element getPropertyElement(TargetConfig config) {
        return null;
    }

    /** Enumeration of supported platforms */
    public enum Platform {
        AUTO,
        ARDUINO,
        NRF52("Nrf52", true),
        RP2040("Rp2040", false),
        LINUX("Linux", true),
        MAC("Darwin", true),
        ZEPHYR("Zephyr", true),
        WINDOWS("Windows", true);

        String cMakeName;

        private boolean multiThreaded = true;

        Platform() {
            this.cMakeName = this.toString();
        }

        Platform(String cMakeName, boolean isMultiThreaded) {
            this.cMakeName = cMakeName;
            this.multiThreaded = isMultiThreaded;
        }

        /** Return the name in lower case. */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }

        /** Get the CMake name for the platform. */
        public String getcMakeName() {
            return this.cMakeName;
        }

        public boolean isMultiThreaded() {
            return this.multiThreaded;
        }
    }

    /**
     * Platform options.
     *
     * @author Anirudh Rengarajan
     */
    public enum PlatformOption implements DictionaryElement {
        NAME("name", PrimitiveType.STRING),
        BAUDRATE("baud-rate", PrimitiveType.NON_NEGATIVE_INTEGER),
        BOARD("board", PrimitiveType.STRING),
        FLASH("flash", PrimitiveType.BOOLEAN),
        PORT("port", PrimitiveType.STRING),
        USER_THREADS("user-threads", PrimitiveType.NON_NEGATIVE_INTEGER);

        public final PrimitiveType type;

        private final String description;

        private PlatformOption(String alias, PrimitiveType type) {
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
