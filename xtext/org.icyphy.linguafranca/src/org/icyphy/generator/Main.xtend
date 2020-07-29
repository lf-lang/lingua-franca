/* Stand-alone version of the Lingua Franca compiler (lfc). */

/*************
Copyright (c) 2020, The University of California at Berkeley.

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

package org.icyphy.generator

import com.google.inject.Inject
import com.google.inject.Provider
import java.io.BufferedReader
import java.io.File
import java.io.FileNotFoundException
import java.io.FileReader
import java.io.IOException
import java.util.Properties
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.GeneratorDelegate
import org.eclipse.xtext.generator.JavaIoFileSystemAccess
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.validation.IResourceValidator
import org.icyphy.LinguaFrancaStandaloneSetup

/**
 * Stand-alone version of the Lingua Franca compiler (lfc).
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>} 
 */
class Main {

    def static main(String[] args) {

        val injector = new LinguaFrancaStandaloneSetup().
            createInjectorAndDoEMFRegistration
        val main = injector.getInstance(Main)
        try {
            val arr = args.get(0).split(" ")
            main.runGenerator(arr.get(0), getProps(arr))
        } catch (FileNotFoundException e) {
            System::err.println('lfc: fatal error: no input file.')
            e.printStackTrace();
            System.exit(1);
        } catch (IOException e) {
            System::err.println('lfc: error reading input file.')
            e.printStackTrace();
            System.exit(1);
        } catch (RuntimeException e) {
            System::err.println('lfc: unexpected error.');
            e.printStackTrace();
            System.exit(1);
        }

    }

    @Inject
    Provider<ResourceSet> resourceSetProvider

    @Inject
    IResourceValidator validator

    @Inject
    GeneratorDelegate generator

    @Inject
    JavaIoFileSystemAccess fileAccess

    /**
     * Store arguments as properties, passed on in the context given to the generator.
     */
    def static protected getProps(String[] args) {
        val len = args.length
        var flags = ""
        val props = new Properties()
        if (len < 2) {
            return props
        } else {
            for (var i = 1; i < len; i++) {
                val k = args.get(i)
                if (k.trim().equals("--no-compile")) {
                    props.setProperty("no-compile", "")
                } else if (k.trim().equals("--target-compiler") &&
                    i + 1 < len) {
                    props.setProperty("target-compiler", args.get(i + 1))
                    i++
                } else {
                    flags = flags.trim + " " + k
                }
            }
        }
        if (flags != "") {
            props.setProperty("target-flags", flags)
        }
        props
    }
    
    /**
     * Load resources, read in the code, parse it, validate it, and, finally,
     * invoke the code generator.
     */
    def protected runGenerator(String string, Properties properties) {
        // Load the resource
        val set = resourceSetProvider.get
        val fileRoot = (new File("")).getAbsolutePath()
        val fileName = fileRoot + File.separator + string;

        val f = new File(fileName)
        if (!f.exists) {
            System::err.println('lfc: error: ' + fileName +
                ': No such file or directory');
            throw new FileNotFoundException(fileName);
        }

        val resource = set.getResource(URI.createFileURI(fileName), true)

        // Read the code
        val code = new StringBuilder();
        val reader = new BufferedReader(new FileReader(fileName));
        var String line;
        while ((line = reader.readLine()) !== null) {
            code.append(line).append("\n");
        }

        // Validate the resource
        val issues = validator.validate(resource, CheckMode.ALL,
            CancelIndicator.NullImpl)
        if (!issues.empty) {
            System::err.println('Aborting. Unable to validate resource.');
            issues.forEach[System.err.println(it)]
            System::exit(1)
        }

        // Configure and start the generator
        fileAccess.outputPath = 'src-gen'
        val context = new StandaloneContext => [
            cancelIndicator = CancelIndicator.NullImpl
            args = properties;
        ]

        generator.generate(resource, fileAccess, context)
        System.out.println('Code generation finished.')
    }
}
