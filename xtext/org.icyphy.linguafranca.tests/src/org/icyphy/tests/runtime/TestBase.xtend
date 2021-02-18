package org.icyphy.tests.runtime

import com.google.inject.Inject
import com.google.inject.Provider
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.PrintStream
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.util.Set
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.diagnostics.Severity
import org.eclipse.xtext.generator.GeneratorDelegate
import org.eclipse.xtext.generator.JavaIoFileSystemAccess
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.extensions.InjectionExtension
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.validation.CheckMode
import org.eclipse.xtext.validation.IResourceValidator
import org.eclipse.xtext.xbase.lib.Functions.Function1
import org.icyphy.Target
import org.icyphy.generator.StandaloneContext
import org.icyphy.tests.LFTest
import org.icyphy.tests.LinguaFrancaInjectorProvider
import org.icyphy.tests.TestRegistry
import org.icyphy.tests.TestRegistry.TestCategory
import org.junit.jupiter.api.^extension.ExtendWith
import java.util.concurrent.TimeUnit
import org.icyphy.tests.LFTest.Result
import org.icyphy.linguaFranca.Reactor
import java.nio.file.Paths
import org.junit.jupiter.api.Test

import static extension org.junit.Assert.assertTrue

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)

class TestBase {
    
    @Inject Provider<ResourceSet> resourceSetProvider
    @Inject IResourceValidator validator
    @Inject GeneratorDelegate generator
    @Inject JavaIoFileSystemAccess fileAccess

    static val out = System.out;
    static val err = System.err;
    
    final static long MAX_EXECUTION_TIME_SECONDS = 30
    
    final static val SHOW_OUTPUT_PASSING_TESTS = false
    
    public final static String DIVIDER = "+---------------------------------------------------------------------------+\n";
    
    public final static String THIN_LINE = "-----------------------------------------------------------------------------\n";
    
    public final static String THICK_LINE = "=============================================================================\n";
    
    public final static String EDGE_LINE = "+---------------------------------------------------------------------------+\n";
    
    protected Target target;
    
    protected boolean check = true;
    
    protected boolean run = true;
    
    protected boolean build = true;
    
    @Test
    def void runGenericTests() {
        printTestHeader("Description: Run generic tests (threads = 0).")
        this.target.runTestsAndPrintResults([
            it === TestCategory.GENERIC
        ], [
            it.properties.setProperty("threads", "0")
            return true
        ])
    }
    
    @Test
    def void runTargetSpecificTests() {
        printTestHeader("Description: Run target-specific tests (threads = 0).")
        this.target.runTestsAndPrintResults([it === TestCategory.TARGET], [
            it.properties.setProperty("threads", "0")
            return true
        ])
    }
    
    @Test
    def void runMultiportTests() {
        printTestHeader("Description: Run multiport tests (threads = 0).")
        this.target.runTestsAndPrintResults([it === TestCategory.MULTIPORT], [
            it.properties.setProperty("threads", "0")
            return true
        ])
    }
    
    @Test
    def void runSingleThreadedTestsAsThreaded() {
        printTestHeader("Description: Run non-threaded and non-federated test (threads = 4).")
        this.target.runTestsAndPrintResults([
            it !== TestCategory.THREADED && it !== TestCategory.FEDERATED
        ], [it.properties.setProperty("threads", "4") return true])
    }

    @Test
    def void runNonFederatedTestsAsFederated() {
        printTestHeader("Description: Run non-federated tests in federated mode.")
        this.target.runTestsAndPrintResults([
            it !== TestCategory.THREADED && it !== TestCategory.FEDERATED
        ], [val r = it.resource.allContents.filter(Reactor).findFirst[it.isMain]
            if (r === null) {
                return false
            }
            r.main = false
            r.federated = true
            return true
        ])
    }
    
    @Test
    def void runThreadedTests() {
        printTestHeader("Description: Run threaded tests.")
        Target.C.runTestsAndPrintResults([it === TestCategory.THREADED], [true])
    }
    
    @Test
    def void runFederatedTests() {
        printTestHeader("Description: Run federated tests.")
        Target.C.runTestsAndPrintResults([it === TestCategory.FEDERATED], [true])
    }

    static def void restoreOutputs() {
        System.out.flush()
        System.err.flush()
        System.setOut(out)
        System.setErr(err)
    }

    static def void redirectOutputs(LFTest test) {
        System.setOut(new PrintStream(test.out))
        System.setErr(new PrintStream(test.err))
//        System.setErr(new PrintStream(new OutputStream() {
//            override write(int b) throws IOException {}
//        }));
    }

    def runTestsAndPrintResults(Target target, Function1<TestCategory, Boolean> selection, Function1<LFTest, Boolean> configuration) {
        val categories = TestCategory.values().filter(selection)
        for (category : categories) {
            println(category.header);
            val tests = TestRegistry.getTests(target, category)
            tests.compileAndRun(configuration)
            println(TestRegistry.getCoverageReport(target, category));
            if (check) {
                tests.checkAndReportFailures
            }
        }
    }
    
    def void printTestHeader(String description) {
        print(TestBase.THICK_LINE)
        println("Target: " + this.target)
        println(description)
        println(TestBase.THICK_LINE)
    }

    def void checkAndReportFailures(Set<LFTest> tests) {
        var passed = tests.filter[!it.hasFailed].size
        print("\n" + THIN_LINE)
        println("Passing: " + passed + "/" + tests.size)
        print(THIN_LINE)
        
        for (test : tests) {
            print(test.reportErrors)
        }
        tests.forall[it.result === Result.TEST_PASS].assertTrue
    }

    def boolean parseAndValidate(LFTest test, Function1<LFTest, Boolean> configuration) {
        // Obtain the resource (i.e., the AST).
        try {
            redirectOutputs(test)
            test.resource = resourceSetProvider.get.getResource(URI.createFileURI(test.path.toString()),true)
            restoreOutputs()
        } catch (Exception e) {
            test.result = Result.PARSE_FAIL
            return false
        }
        
        if (test.resource === null || !test.resource.errors.isEmpty) {
            test.result = Result.PARSE_FAIL
            return false
        }
        // Update the test by applying the configuration. E.g., to carry out an AST transformation.
        if (!configuration.apply(test)) {
            test.result = Result.CONFIG_FAIL
            return false
        }
        
        if (!this.build) {
            test.properties.setProperty("no-compile", "")
        }
        
        if (!test.resource.allContents.filter(Reactor).exists[it.isMain || it.isFederated]) {
            test.result = Result.NO_MAIN_FAIL
            return false
        }
        
        // Validate the resource and store issues in the test object.
        try {
            redirectOutputs(test)
            val issues = validator.validate(test.resource, CheckMode.ALL, CancelIndicator.NullImpl)
            if (!issues.isNullOrEmpty) {
                test.compilationIssues = issues.join("\n")
                if (issues.exists [
                    it.severity == Severity.ERROR
                ]) {
                    restoreOutputs()
                    test.result = Result.VALIDATE_FAIL
                    return false
                    //test.executionErrors = "Did not execute due to compilation error(s)."
                }
            }
        } catch(Exception e) {
            test.result = Result.VALIDATE_FAIL
            restoreOutputs()
            return false
        }
        restoreOutputs()
        return true
    }

    def boolean generateCode(LFTest test) {
        if (test.resource !== null) {
            fileAccess.outputPath = 'src-gen'
            val context = new StandaloneContext => [
                cancelIndicator = CancelIndicator.NullImpl
                args = test.properties;
            ]
            redirectOutputs(test)
            try {
                generator.generate(test.resource, fileAccess, context)
                // FIXME: if the target compiler reports errors, we should receive an exception.
                // This is currently not the case.
            } catch (Exception e) {
                test.compilationIssues += e.toString() + "\n"
                // FIXME: Weed out kinds of exceptions and set result accordingly.
                return false
            }
            
            restoreOutputs()
            //println(test.out.toString(StandardCharsets.UTF_8))
            return true
        }
        return false
    }

    def execute(LFTest test) {
        var ProcessBuilder pb;
        val nameWithExtension = test.path.fileName.toString
        val nameOnly = nameWithExtension.substring(0, nameWithExtension.lastIndexOf('.'))
        
        // FIXME: we probably want to use the createCommand utility from GeneratorBase here.
        // Should it be made static? Factored out?
        
        switch(test.target) {
            case C,
            case CCPP: {
                val bin = test.path.getParent.resolve("bin") // Relative to source
                val file = bin.resolve(nameOnly)
                if (Files.exists(file)) {
                    pb = new ProcessBuilder(file.toString)
                } else {
                    test.result = Result.NO_EXEC_FAIL
                }
            }
            case CPP: {
                val bin = Paths.get(Paths.get("").toAbsolutePath() + File.separator + "bin") // CWD 
                val file = bin.resolve(nameOnly)
                if (Files.exists(file)) {
                    pb = new ProcessBuilder(file.toString)
                } else {
                    test.result = Result.NO_EXEC_FAIL
                    //println(">>>>>" + file)
                    // FIXME: add message
                }
            }
            
            case Python: {
                val bin = test.path.getParent.resolve("src-gen") // Relative to source
                val file = bin.resolve(nameOnly + ".py")
                if (Files.exists(file)) {
                    pb = pb = new ProcessBuilder("python", file.toString)
                } else {
                    test.result = Result.NO_EXEC_FAIL
                    //println(">>>>>" + file)
                    // FIXME: add message
                }
            }
            case TS: {
                val bin = test.path.getParent.resolve(nameOnly + File.separator + "dist") // Relative to source
                val file = bin.resolve(nameOnly + ".js")
                if (Files.exists(file)) {
                    pb = pb = new ProcessBuilder("node", file.toString)
                } else {
                    test.result = Result.NO_EXEC_FAIL
                    println(">>>>>" + file)
                    // FIXME: add message
                }
            }
        }
        if (pb !== null) {
            try {
                val p = pb.start()
                //println("Executing: " + nameOnly)
                if(!p.waitFor(MAX_EXECUTION_TIME_SECONDS, TimeUnit.SECONDS)) {
                    p.destroyForcibly();
                    test.result = Result.TEST_TIMEOUT
                } else {
                    //println(new String(p.getInputStream().readAllBytes()))
                    if (p.exitValue == 0) {
                        test.result = Result.TEST_PASS
                    } else {
                        test.result = Result.TEST_FAIL
                    }
                }
                
            } catch(Exception e) {
                test.result = Result.TEST_FAIL
            }
            
        }

    }

    def compileAndRun(Set<LFTest> tests, Function1<LFTest, Boolean> configuration) {
        val x = 78f / tests.size()
        var marks = 0
        var done = 0
        for (test : tests) {
            if (test.parseAndValidate(configuration) && test.generateCode()) {
                if (run) {
                    test.execute()
                }
            }
            while (Math.floor(done * x) > marks && marks < 78) {
                print("=")
                marks++
            }
            
            done++
        }
//        for (var i=0; i< 78-j; i++) {
//            print("=")
//        }
        print("\n")
    }
}
