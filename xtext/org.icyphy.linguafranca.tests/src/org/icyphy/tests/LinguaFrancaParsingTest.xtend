package org.icyphy.tests

import com.google.inject.Inject
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.icyphy.linguaFranca.Model
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.^extension.ExtendWith

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)

/**
 * Test harness for ensuring that grammar captures
 * all corner cases.
 */
class LinguaFrancaParsingTest {
    @Inject extension ParseHelper<Model>

    @Test
    def void checkForTarget() {
        val result = '''
            targett C;
            reactor Foo {
            }
        '''.parse
        Assertions.assertNotNull(result)
        val errors = result.eResource.errors
        Assertions.assertFalse(errors.isEmpty, "Failed to catch misspelled target keyword.")
    }
}