package org.lflang.tests;

import org.eclipse.xtext.testing.InjectWith;
import org.eclipse.xtext.testing.extensions.InjectionExtension;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * Base class for test classes that can use `com.google.inject.Inject` annotations to provide
 * dependencies.
 *
 * @author Clément Fournier
 * @ingroup Tests
 */
@ExtendWith(InjectionExtension.class)
@InjectWith(LFInjectorProvider.class)
public abstract class LfInjectedTestBase {}
