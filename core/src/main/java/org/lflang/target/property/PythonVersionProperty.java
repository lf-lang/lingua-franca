package org.lflang.target.property;

/**
 * Directive for specifying a specific version of the reactor runtime library.
 */
public final class PythonVersionProperty extends StringProperty {

    /** Singleton target property instance. */
    public static final PythonVersionProperty INSTANCE = new PythonVersionProperty();

    private PythonVersionProperty() {
        super();
    }

    @Override
    public String name() {
        return "python-version";
    }
}
