package org.lflang.validation;

import java.util.List;

public abstract class LFValidator extends AbstractLFValidator {

    public abstract List<String> getTargetPropertyErrors();
    // todo port xtend into new kotlin file
}
