/* Scoping unit tests. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.icyphy.tests.compiler

import com.google.inject.Inject
import com.google.inject.Provider
import java.io.File
import java.util.Properties
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.GeneratorDelegate
import org.eclipse.xtext.generator.JavaIoFileSystemAccess
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.validation.IResourceValidator
import org.icyphy.Target
import org.icyphy.generator.StandaloneContext
import org.icyphy.linguaFranca.Model
import org.icyphy.tests.LinguaFrancaInjectorProvider
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith

import static org.junit.Assert.assertTrue
import org.icyphy.tests.LinguaFrancaTestHelper

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)

/**
 * Collection of unit tests to ensure validation is done correctly.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class LinguaFrancaCoverageTest {
    @Inject extension ParseHelper<Model>
    
    @Inject Provider<ResourceSet> resourceSetProvider
    @Inject IResourceValidator validator
    @Inject GeneratorDelegate generator

    @Inject JavaIoFileSystemAccess fileAccess
    
    
    //@Inject GeneratorDelegate generator
    /**
     * Helper function to parse a Lingua Franca program and expect no errors.
     * @return A model representing the parsed string.
     */
    def parseWithoutError(String s) {
        val model = s.parse
        Assertions.assertNotNull(model)
        Assertions.assertTrue(model.eResource.errors.isEmpty,
            "Encountered unexpected error while parsing: " +
                model.eResource.errors)
        return model
    }
    
    /**
     * Helper function to parse a Lingua Franca program and expect errors.
     * @return A model representing the parsed string.
     */
    def parseWithError(String s) {
        val model = s.parse
        Assertions.assertNotNull(model)
        Assertions.assertFalse(model.eResource.errors.isEmpty)
        return model
    } 

    /**
     * Ensure that duplicate identifiers for actions reported.
     */
    @Test
    def void generateCodeForAllIntegrationTests() {
        val set = resourceSetProvider.get
        for (target : Target.values) {
            val location = LinguaFrancaTestHelper.LF_TEST_PATH + target
            val d = new File(location)
            if (d.exists) {
                println("Target: " + target)
                d.list.filter[it.endsWith(".lf")].forEach [
                    println(it)
                    val path = location + File.separator + it
                    val resource = set.getResource(URI.createFileURI(path),
                        true)
                    val issues = validator.validate(resource, CheckMode.ALL,
                        CancelIndicator.NullImpl)
                    issues.forEach[println(it)]
                    assertTrue(
                        "Ensure that all integration tests yield no errors or warnings.",
                        issues.isEmpty)

                    fileAccess.outputPath = 'src-gen'
                    val properties = new Properties()
                    properties.setProperty("no-compile", "")
                    val context = new StandaloneContext => [
                        cancelIndicator = CancelIndicator.NullImpl
                        args = properties;
                    ]
                    // By doing this, we also get code coverage in the generator package.
                    generator.generate(resource, fileAccess, context)

                ]
            }
        }
    }
}
