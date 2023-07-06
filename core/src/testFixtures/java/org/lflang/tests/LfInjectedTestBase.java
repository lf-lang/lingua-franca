package org.lflang.tests;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * Base class for test classes that can use {@link com.google.inject.Inject} annotations to provide
 * dependencies.
 *
 * @author Clément Fournier
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public abstract class LfInjectedTestBase {}
