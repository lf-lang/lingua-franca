/*
 * generated by Xtext 2.18.0
 */
package org.icyphy.generator

import com.google.inject.Inject
import com.google.inject.Provider
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.io.IOException
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.GeneratorContext
import org.eclipse.xtext.generator.GeneratorDelegate
import org.eclipse.xtext.generator.JavaIoFileSystemAccess
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.validation.IResourceValidator
import org.icyphy.LinguaFrancaStandaloneSetup
import java.io.FileNotFoundException

class Main {

	def static main(String[] args) {
		// FIXME: perhaps accept a `-o` flag here
		
		val injector = new LinguaFrancaStandaloneSetup().createInjectorAndDoEMFRegistration
		val main = injector.getInstance(Main)
		try {
		    main.runGenerator(args.get(0))    
		} catch(FileNotFoundException e) {
		    System::err.println('lfc: fatal error: no input file')
		    System.exit(1);
		} catch(IOException e) {
            System::err.println('lfc: error reading input file');
            System.exit(1);
        } catch(RuntimeException e) {
            System::err.println('lfc: unexpected error');
            System::err.println(e.message);
            System.exit(1);
        }
		
	}

	@Inject Provider<ResourceSet> resourceSetProvider

	@Inject IResourceValidator validator

	@Inject GeneratorDelegate generator

	@Inject JavaIoFileSystemAccess fileAccess

	def protected runGenerator(String string) {
		// Load the resource
		val set = resourceSetProvider.get
		val fileRoot = (new File("")).getAbsolutePath()
		val fileName = fileRoot + File.separator + string;

        val f = new File(fileName)
        if (!f.exists) {
            System::err.println('lfc: error: ' + fileName + ': No such file or directory');
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
		val issues = validator.validate(resource, CheckMode.ALL, CancelIndicator.NullImpl)
		if (!issues.empty) {
			System::err.println('Aborting. Unable to validate resource.');
			issues.forEach[System.err.println(it)]
			return
		}

		// Configure and start the generator
		fileAccess.outputPath = 'src-gen'
		val context = new GeneratorContext => [
			cancelIndicator = CancelIndicator.NullImpl
		]
		// FIXME: perhaps use context to pass in additional arguments?
		generator.generate(resource, fileAccess, context)
		System.out.println('Code generation finished.')
	}
}
