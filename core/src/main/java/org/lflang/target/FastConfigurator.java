package org.lflang.target;

import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.validation.LFValidator.ValidationReporter;

public class FastConfigurator implements TargetPropertyConfig<Boolean> {

    @Override
    public void parseIntoTargetConfig(TargetConfig config, Element value, MessageReporter err) {
        config.fastMode = this.parse(value);
    }

    @Override
    public Boolean parse(Element value) {
        return ASTUtils.toBoolean(value);
    }

    @Override
    public void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) {
        if (pair != null) {
            // Check for federated
            for (Reactor reactor : ast.getReactors()) {
                // Check to see if the program has a federated reactor
                if (reactor.isFederated()) {
                    reporter.error(
                        "The fast target property is incompatible with federated programs.",
                        pair,
                        Literals.KEY_VALUE_PAIR__NAME);
                    break;
                }
            }

            // Check for physical actions
            for (Reactor reactor : ast.getReactors()) {
                // Check to see if the program has a physical action in a reactor
                for (Action action : reactor.getActions()) {
                    if (action.getOrigin().equals(ActionOrigin.PHYSICAL)) {
                        reporter.error(
                            "The fast target property is incompatible with physical actions.",
                            pair,
                            Literals.KEY_VALUE_PAIR__NAME);
                        break;
                    }
                }
            }
        }

    }

    @Override
    public Element getPropertyElement(TargetConfig config) {
        return ASTUtils.toElement(config.fastMode);
    }
}
