package org.lflang.generator;

import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.ASTUtils;
import org.lflang.TimeValue;
import org.lflang.lf.Assignment;
import org.lflang.lf.Delay;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Parameter;
import org.lflang.lf.StateVar;
import org.lflang.lf.Time;
import org.lflang.TimeUnit;
import org.lflang.lf.Value;

/**
 * Encapsulates logic for representing {@code Value}s in a
 * target language.
 */
public final class ValueGenerator {

    /**
     * A {@code TimeInTargetLanguage} is a
     * target-language-specific time representation
     * strategy.
     */
    public interface TimeInTargetLanguage {
        String apply(TimeValue t);
    }

    /**
     * A {@code GetTargetReference} instance is a
     * target-language-specific function. It provides the
     * target language code that refers to the given
     * parameter {@code param}.
     */
    public interface GetTargetReference {
        String apply(Parameter param);
    }

    private final TimeInTargetLanguage timeInTargetLanguage;
    private final GetTargetReference getTargetReference;

    /**
     * Instantiates a target-language-specific
     * ValueGenerator parameterized by {@code f}.
     * @param f a time representation strategy
     */
    public ValueGenerator(TimeInTargetLanguage f, GetTargetReference g) {
        this.timeInTargetLanguage = f;
        this.getTargetReference = g;
    }

    /**
     * Create a list of state initializers in target code.
     *
     * @param state The state variable to create initializers for
     * @return A list of initializers in target code
     */
    public List<String> getInitializerList(StateVar state) {
        List<String> list = new ArrayList<>();
        // FIXME: Previously, we returned null if it was not initialized, which would have caused an
        //  NPE in TSStateGenerator. Is this the desired behavior?
        if (!ASTUtils.isInitialized(state)) return list;
        for (Value v : state.getInit()) {
            if (v.getParameter() != null) {
                list.add(getTargetReference.apply(v.getParameter()));
            } else {
                list.add(getTargetValue(v, ASTUtils.isOfTimeType(state)));
            }
        }
        return list;
    }

    /**
     * Create a list of default parameter initializers in target code.
     *
     * @param param The parameter to create initializers for
     * @return A list of initializers in target code
     */
    public List<String> getInitializerList(Parameter param) {
        List<String> list = new ArrayList<>();
        if (param == null) return list;
        for (Value v : param.getInit())
            list.add(getTargetValue(v, ASTUtils.isOfTimeType(param)));
        return list;
    }

    /**
     * Create a list of parameter initializers in target code in the context
     * of an reactor instantiation.
     *
     * This respects the parameter assignments given in the reactor
     * instantiation and falls back to the reactors default initializers
     * if no value is assigned to it.
     *
     * @param param The parameter to create initializers for
     * @return A list of initializers in target code
     */
    public List<String> getInitializerList(Parameter param, Instantiation i) {
        List<Assignment> assignments = i.getParameters().stream()
                                        .filter(it -> it.getLhs() == param)
                                        .collect(Collectors.toList());
        if (assignments.isEmpty())  // Case 0: The parameter was not overwritten in the instantiation
            return getInitializerList(param);
        // Case 1: The parameter was overwritten in the instantiation
        List<String> list = new ArrayList<>();
        if (assignments.get(0) == null) return list;
        for (Value init : assignments.get(0).getRhs())
            list.add(getTargetValue(init, ASTUtils.isOfTimeType(param)));
        return list;
    }

    /**
     * Return the time specified by {@code t}, expressed as
     * code that is valid for some target languages.
     */
    public String getTargetTime(TimeValue t) {
        return timeInTargetLanguage.apply(t);
    }

    /**
     * Return the time specified by {@code t}, expressed as
     * code that is valid for some target languages.
     */
    public String getTargetTime(Time t) {
        return timeInTargetLanguage.apply(new TimeValue(t.getInterval(), TimeUnit.fromName(t.getUnit())));
    }

    /**
     * Return the time specified by {@code d}, expressed as
     * code that is valid for some target languages.
     */
    public String getTargetTime(Delay d) {
        return d.getParameter() != null ? ASTUtils.toText(d) : timeInTargetLanguage.apply(
            ASTUtils.toTimeValue(d.getTime())  // The time is given as a parameter reference.
        );
    }

    /**
     * Return the time specified by {@code v}, expressed as
     * code that is valid for some target languages.
     */
    public String getTargetTime(Value v) {
        return getTargetValue(v, true);
    }

    /**
     * Get textual representation of a value in the target language.
     *
     * If the value evaluates to 0, it is interpreted as a normal value.
     *
     * @param v A time AST node
     * @return A time string in the target language
     */
    public String getTargetValue(Value v) {
        return ASTUtils.toText(v);
    }

    /**
     * Get textual representation of a value in the target language.
     *
     * @param v A time AST node
     * @param isTime Whether {@code v} is expected to be a time
     * @return A time string in the target language
     */
    public String getTargetValue(Value v, boolean isTime) {
        if (v.getTime() != null) return getTargetTime(v.getTime());
        if (isTime && ASTUtils.isZero(v)) return timeInTargetLanguage.apply(TimeValue.ZERO);
        return ASTUtils.toText(v);
    }
}
