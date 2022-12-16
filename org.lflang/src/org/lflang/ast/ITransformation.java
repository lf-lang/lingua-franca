package org.lflang.ast;

import java.util.List;

import org.lflang.generator.LFResource;
import org.lflang.lf.Reactor;

/**
 * Interface for AST Transfomations
 */
public interface ITransformation {

    /**
     * Apply the AST transformation to all given reactors.
     */
    void applyTransformation(List<Reactor> reactors);
}
