package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.lflang.Target;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.ClockSyncConfigurator.ClockSyncMode;
import org.lflang.target.CoordinationConfig.CoordinationType;
import org.lflang.target.LoggingConfigurator.LogLevel;
import org.lflang.target.PlatformConfigurator.Platform;
import org.lflang.target.SchedulerConfigurator.SchedulerOption;
import org.lflang.target.property.BuildConfig.BuildType;
import org.lflang.validation.LFValidator;

/**
 * A type that can assume one of several types.
 *
 * @author Marten Lohstroh
 */
public enum UnionType implements TargetPropertyType {
    STRING_OR_STRING_ARRAY(Arrays.asList(PrimitiveType.STRING, ArrayType.STRING_ARRAY), null),
    PLATFORM_STRING_OR_DICTIONARY(
        Arrays.asList(PrimitiveType.STRING, DictionaryType.PLATFORM_DICT), null),
    FILE_OR_FILE_ARRAY(Arrays.asList(PrimitiveType.FILE, ArrayType.FILE_ARRAY), null),
    BUILD_TYPE_UNION(Arrays.asList(BuildType.values()), null),
    COORDINATION_UNION(Arrays.asList(CoordinationType.values()), CoordinationType.CENTRALIZED),
    SCHEDULER_UNION(Arrays.asList(SchedulerOption.values()), SchedulerOption.getDefault()),
    LOGGING_UNION(Arrays.asList(LogLevel.values()), LogLevel.INFO),
    PLATFORM_UNION(Arrays.asList(Platform.values()), Platform.AUTO),
    CLOCK_SYNC_UNION(Arrays.asList(ClockSyncMode.values()), ClockSyncMode.INIT),
    DOCKER_UNION(Arrays.asList(PrimitiveType.BOOLEAN, DictionaryType.DOCKER_DICT), null),
    TRACING_UNION(Arrays.asList(PrimitiveType.BOOLEAN, DictionaryType.TRACING_DICT), null);

    /** The constituents of this type union. */
    public final List<Enum<?>> options;

    /** The default type, if there is one. */
    private final Enum<?> defaultOption;

    /**
     * Private constructor for creating unions types.
     *
     * @param options The types that that are part of the union.
     * @param defaultOption The default type.
     */
    private UnionType(List<Enum<?>> options, Enum<?> defaultOption) {
        this.options = options;
        this.defaultOption = defaultOption;
    }

    /**
     * Return the type among those in this type union that matches the given name.
     *
     * @param name The string to match against.
     * @return The matching dictionary element (or null if there is none).
     */
    public Enum<?> forName(String name) {
        return Target.match(name, options);
    }

    /** Recursively check that the passed in element conforms to the rules of this union. */
    @Override
    public void check(Element e, String name, LFValidator v) {
        Optional<Enum<?>> match = this.match(e);
        if (match.isPresent()) {
            // Go deeper if the element is an array or dictionary.
            Enum<?> type = match.get();
            if (type instanceof DictionaryType) {
                ((DictionaryType) type).check(e, name, v);
            } else if (type instanceof ArrayType) {
                ((ArrayType) type).check(e, name, v);
            } else if (type instanceof PrimitiveType) {
                ((PrimitiveType) type).check(e, name, v);
            } else if (!(type instanceof Enum<?>)) {
                throw new RuntimeException("Encountered an unknown type.");
            }
        } else {
            // No match found; report error.
            TargetPropertyType.produceError(name, this.toString(), v);
        }
    }

    /**
     * Internal method for matching a given element against the allowable types.
     *
     * @param e AST node that represents the value of a target property.
     * @return The matching type wrapped in an Optional object.
     */
    private Optional<Enum<?>> match(Element e) {
        return this.options.stream()
            .filter(
                option -> {
                    if (option instanceof TargetPropertyType) {
                        return ((TargetPropertyType) option).validate(e);
                    } else {
                        return ASTUtils.elementToSingleString(e).equalsIgnoreCase(option.toString());
                    }
                })
            .findAny();
    }

    /**
     * Return true if this union has an option that matches the given element.
     *
     * @param e The element to match against this type.
     */
    @Override
    public boolean validate(Element e) {
        if (this.match(e).isPresent()) {
            return true;
        }
        return false;
    }

    /**
     * Return a human-readable description of this type. If three is a default option, then indicate
     * it.
     */
    @Override
    public String toString() {
        return "one of the following: "
            + options.stream()
            .map(
                option -> {
                    if (option == this.defaultOption) {
                        return option.toString() + " (default)";
                    } else {
                        return option.toString();
                    }
                })
            .collect(Collectors.joining(", "));
    }
}