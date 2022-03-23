package org.lflang.generator;

import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.TimeValue;
import org.lflang.lf.Assignment;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.StateVar;
import org.lflang.lf.Time;
import org.lflang.TimeUnit;

/**
 * Encapsulates logic for representing {@code Value}s in a
 * target language.
 */
public final class ExpressionGenerator {

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
     * ExpressionGenerator parameterized by {@code f}.
     * @param f a time representation strategy
     */
    public ExpressionGenerator(TimeInTargetLanguage f, GetTargetReference g) {
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
        for (Expression expr : state.getInit()) {
            if (expr instanceof  ParameterReference) {
                list.add(getTargetReference.apply(((ParameterReference)expr).getParameter()));
            } else {
                list.add(getTargetValue(expr, ASTUtils.isOfTimeType(state)));
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
        for (Expression expr : param.getInit())
            list.add(getTargetValue(expr, ASTUtils.isOfTimeType(param)));
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
        for (Expression expr : assignments.get(0).getRhs())
            list.add(getTargetValue(expr, ASTUtils.isOfTimeType(param)));
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
     * Return the time specified by {@code v}, expressed as
     * code that is valid for some target languages.
     */
    public String getTargetTime(Expression expr) {
        return getTargetValue(expr, true);
    }

    /**
     * Get textual representation of an expression in the target language.
     *
     * If the value evaluates to 0, it is interpreted as a literal.
     *
     * @param expr A time AST node
     * @return A time string in the target language
     */
    public String getTargetValue(Expression expr) {
        return ASTUtils.toText(expr);
    }

    /**
     * Get textual representation of an expression in the target language.
     *
     * @param expr A time AST node
     * @param isTime Whether {@code v} is expected to be a time
     * @return A time string in the target language
     */
    public String getTargetValue(Expression expr, boolean isTime) {
        if (expr instanceof Time) return getTargetTime((Time)expr);
        if (isTime && ASTUtils.isZero(expr)) return timeInTargetLanguage.apply(TimeValue.ZERO);
        return ASTUtils.toText(expr);
    }
}
