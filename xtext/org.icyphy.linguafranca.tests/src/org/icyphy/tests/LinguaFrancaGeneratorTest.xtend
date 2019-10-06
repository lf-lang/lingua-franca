/*
 * Tests for the Lingua Franca code generator.
 */
package org.icyphy.tests

import com.google.inject.Inject
import com.google.inject.Provider
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.io.IOException
import java.io.InputStream
import java.io.InputStreamReader
import java.util.LinkedList
import java.util.regex.Pattern
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.GeneratorContext
import org.eclipse.xtext.generator.JavaIoFileSystemAccess
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.XtextRunner
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.testing.util.ParseHelper
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.validation.IResourceValidator
import org.icyphy.generator.LinguaFrancaGenerator
import org.icyphy.linguaFranca.Model
import org.junit.Test
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.^extension.ExtendWith
import org.junit.runner.RunWith

@RunWith(XtextRunner)
@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)
class LinguaFrancaGeneratorTest {
	@Inject
	ParseHelper<Model> parseHelper
	
	@Inject
	LinguaFrancaGenerator generator
	
	@Inject
	Provider<ResourceSet> resourceSetProvider
	
	@Inject
	IResourceValidator validator
	
	@Inject
	JavaIoFileSystemAccess fileAccess
	
	@Test
	def void checkCTestModels() {
		var target = "C"
		var testFiles = readTestFiles(target)
		Assertions.assertNotNull(testFiles, "Couldn't find testFiles.txt file for target: " + target)
		
		var errors = new LinkedList<String>()
		var testCount = 0

		for (file: testFiles) {
			testCount++
			compileAndRun(target, file, errors)
		}
		var message = '''Errors: «errors.length» out of «testCount» tests:
* «errors.join("\n* ")»'''
		if (!errors.isEmpty) {
			println("FAILURE:\n" + message)
		}
		Assertions.assertTrue(errors.isEmpty, message)
		println('''SUCCESS. Number of compile and run tests: «testCount»''')
	}
	
	/** Compile the specified test file for the specified target
	 *  and append any encountered errors to the specified list of errors.
	 *  @param target The target.
	 *  @param file The Lingua Franca file name.
	 *  @param directory The directory into which to write temporary files.
	 *  @param errors A list to which to append errors.
	 */
	private def compileAndRun(
		String target, String file, LinkedList<String> errors
	) {        
        val set = resourceSetProvider.get
        // Get an absolute path to the file.
        // Current directory is lingua-franca/xtext/org.icyphy.linguafranca.tests
        // We need to be in lingua-franca/xtext/org.icyphy.linguafranca/src/test/C
        val fileRoot = (new File("")).getAbsolutePath()
        val srcPath = fileRoot
                + File.separator
                + ".."
                + File.separator
                + "org.icyphy.linguafranca"
                + File.separator
                + "src"
                + File.separator
                + "test"
                + File.separator
                + target
        val fileName = srcPath
                + File.separator
                + file;
        val resource = set.getResource(URI.createFileURI(fileName), true)

        var code = new StringBuilder()
        try {
            // The following reads a file relative to the classpath.
            // The file needs to be in the src directory.
            var reader = new BufferedReader(new FileReader(fileName))
            var line = ""
            while ((line = reader.readLine()) !== null) {
                code.append(line).append("\n");
            }
        } catch (IOException e) {
            System::err.println('Aborting. Unable to read file: ' + fileName);
            return;
        }
        
        // Add imports to the resources, as done in Main.xtend.
        // Parse out imports and add them to a list
        // RegEx based on org.eclipse.xtext.common.Terminals
        val id = "(?:([a-z]|[A-Z]|_)\\w*)";
        // RegEx based on LinguaFranca.xtext
        val pattern = Pattern.compile("import(?:\\s)*(" + id + "(?:." + id + ")*)\\s*;");
        val matcher = pattern.matcher(code);

        val imports = newArrayList();
        while (matcher.find) {
            imports.add(matcher.group(1));
        }
        // Add the listed imports to the resource
        // FIXME: This doesn't work! There are no imports.
        for (import : imports) {
            var importPath = srcPath + File.separator + import
            set.getResource(URI.createFileURI(importPath), true)
        }
        
        // Validate the resource
        val issues = validator.validate(resource, CheckMode.ALL, CancelIndicator.NullImpl)
        if (!issues.empty) {
            System::err.println('Aborting. Unable to validate resource.');
            issues.forEach[System.err.println(it)]
            // Add to the errors list so that this counts as a failed test.
            issues.forEach[errors.add("ERROR: Validation failed on " + fileName + "\n"
                + it.toString
            )]
            return
        }
        
        
        // Check that the file parses.
        // FIXME: Needed?
        println("*** Parsing test file: " + file)
        val parsed = parseHelper.parse(code)
        if (parsed === null) {
            errors.add('''Parser returned null on file «file».''')
            return
        }
        val parseErrors = parsed.eResource.errors
        if (!parseErrors.isEmpty) {
            errors.add('''Parse errors in «file»:
*** «parseErrors.join("\n*** ")»''')
            return
        }
        
        // Generate code.
        println("Generating code for test file: " + file)
                
        // Configure and start the generator
        val context = new GeneratorContext => [
            cancelIndicator = CancelIndicator.NullImpl
        ]
        // Specify that output should go into src-gen.
        fileAccess.outputPath = 'src-gen'
        generator.doGenerate(resource, fileAccess, context)
               	
       	// Run the generated code using a provided run command, if it is provided,
       	// and a default otherwise.
   		// Construct the output filename.
   		var outputFile = file.substring(0, file.length - 3)
       	// By default, limit tests to 10 seconds.
       	var runCommand = newArrayList("bin/" + outputFile, "-timeout", "10", "secs")
       	var runCommandOverridden = false;
       	var threads = ""
   		if (parsed.target.parameters !== null) {
   			for (parameter: parsed.target.parameters.assignments) {
   				if (parameter.name.equals("run")) {
    				// Strip off enclosing quotation marks and split at spaces.
   					val command = parameter.value.substring(1, parameter.value.length - 1).split(' ')
   					runCommand.clear
   					runCommand.addAll(command)
   					runCommandOverridden = true
   				} else if (parameter.name.equals("threads")) {
   					threads = parameter.value
				}
   			}
   		}
   		if (!runCommandOverridden && !threads.equals("")) {
   			runCommand.add("-threads")
   			runCommand.add(threads)
   		}
   		// Run the generated code.
        println("In directory: " + srcPath)
        println("Running with command: " + runCommand.join(" "))
        try {
        var builder = new ProcessBuilder(runCommand)
        builder.directory(new File(srcPath));
        var process = builder.start()
        var stdout = readStream(process.getInputStream())
        var stderr = readStream(process.getErrorStream())
        if (stdout.length() > 0) {
            println("--- Standard output:")
            println(stdout)
            println("--- End standard output.")
        }
        process.waitFor() // FIXME: we should probably add a timeout here
        if (process.exitValue !== 0 || stderr.length() > 0) {
            errors.add("ERROR running: " + runCommand.join(" ")
                + "\nExecution returned with error code: " + process.exitValue
                + "\n"
                + stderr.toString)
        } else if (process.exitValue === 0) {
            println("SUCCESS running the generated code.")
        }
        
        } catch (Exception ex) {
            errors.add("FAILED to run: " + runCommand.join(" ")
                + "Exception:\n" + ex)
        }
   	}
	
	/** Read the specified input stream until an end of file is encountered
	 *  and return the result as a StringBuilder.
	 *  @param stream The stream to read.
	 *  @return The result as a string.
	 */
	private def readStream(InputStream stream) {
		var reader = new BufferedReader(new InputStreamReader(stream))
		var result = new StringBuilder();
		var line = "";
		while ( (line = reader.readLine()) !== null) {
   			result.append(line);
   			result.append(System.getProperty("line.separator"));
		}
		stream.close()
		reader.close()
		result
	}
	
	/** Read the "testFiles.txt" file in the test directory for the specified
	 *  target language and return a list of the filenames for tests in that
	 *  directory.
	 *  @param target The target name.
	 *  @return A list of test files, or null if the testFiles.txt file was not found.
	 */
	private def readTestFiles(String target) {
		var inputStream = this.class.getResourceAsStream("/test/" + target + "/testFiles.txt")
		if (inputStream === null) {
			return null
		}
		try {
 			var result = new LinkedList<String>()
			// The following reads a file relative to the classpath.
			// The file needs to be in the src directory.
    		var reader = new BufferedReader(new InputStreamReader(inputStream))
        	var line = ""
        	while ((line = reader.readLine()) !== null) {
            	result.add(line);
        	}
			return result
        } finally {
        	inputStream.close
        }
	}
}
