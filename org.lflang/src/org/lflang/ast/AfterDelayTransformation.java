package org.lflang.ast;

import java.util.List;

import org.lflang.ASTUtils;
import org.lflang.generator.IDelayBodyGenerator;
import org.lflang.generator.TargetTypes;
import org.lflang.lf.Reactor;

public class AfterDelayTransformation implements ITransformation {

    /**
     * A code generator used to insert reaction bodies for the generated delay reactors.
     */
    private IDelayBodyGenerator delayCodeGenerator;

    /**
     * A target type instance that is used during the transformation to manage target specific types
     */
    private TargetTypes targetTypes;

    public AfterDelayTransformation(IDelayBodyGenerator generator, TargetTypes targetTypes) {
        this.delayCodeGenerator = generator;
        this.targetTypes = targetTypes;
    }

    /**
     * Transform all after delay connections by inserting generated delay reactors.
     */
    @Override
    public void applyTransformation(List<Reactor> reactors) {
        ASTUtils.insertGeneratedDelays(reactors, delayCodeGenerator, targetTypes);
    }
}
