package org.icyphy.tests.runtime

import com.google.inject.Inject
import com.google.inject.Provider
import java.io.File
import java.io.PrintStream
import java.nio.file.Files
import java.nio.file.Paths
import java.util.Set
import java.util.concurrent.TimeUnit
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
import org.icyphy.ASTUtils
import org.icyphy.Target
import org.icyphy.generator.StandaloneContext
import org.icyphy.linguaFranca.Reactor
import org.icyphy.tests.LFTest
import org.icyphy.tests.LFTest.Result
import org.icyphy.tests.LinguaFrancaInjectorProvider
import org.icyphy.tests.TestRegistry
import org.icyphy.tests.TestRegistry.TestCategory
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.^extension.ExtendWith

import static extension org.junit.Assert.assertTrue
import org.icyphy.generator.GeneratorBase
import org.icyphy.TargetConfig
import org.icyphy.FileConfig

@ExtendWith(InjectionExtension)
@InjectWith(LinguaFrancaInjectorProvider)

abstract class TestBase {
    
    @Inject Provider<ResourceSet> resourceSetProvider
    @Inject IResourceValidator validator
    @Inject GeneratorDelegate generator
    @Inject JavaIoFileSystemAccess fileAccess

    static val out = System.out;
    static val err = System.err;
    
    final static long MAX_EXECUTION_TIME_SECONDS = 60
    
    public final static String NEW_LINE = System.getProperty("line.separator");
    
    public final static String DIVIDER = "+----------------------------------------------------------------------------+" + NEW_LINE;
    
    public final static String THIN_LINE = "------------------------------------------------------------------------------" + NEW_LINE;
    
    public final static String THICK_LINE = "==============================================================================" + NEW_LINE;
    
    public final static String EDGE_LINE = "+--------------------------------------------------------------------=-------+" + NEW_LINE;
    
    protected Target target;
    
    protected boolean check = true;
    
    protected boolean run = true;
    
    protected boolean build = true;
    
    def getRoot() {
        return TestRegistry.LF_TEST_PATH + target
    }
    
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
    def void runNonFederatedTestsAsFederated() {
        printTestHeader("Description: Run non-federated tests in federated mode.")
        this.target.runTestsAndPrintResults([
            it !== TestCategory.CONCURRENT && it !== TestCategory.FEDERATED
        ], [ASTUtils.makeFederated(it.fileConfig.resource)])
    }
    
    @Test
    def void runThreadedTests() {
        printTestHeader("Description: Run threaded tests.")
        this.target.runTestsAndPrintResults([it === TestCategory.CONCURRENT], [true])
    }
    
    @Test
    def void runFederatedTests() {
        printTestHeader("Description: Run federated tests.")
        this.target.runTestsAndPrintResults([it === TestCategory.FEDERATED], [true])
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
        print(NEW_LINE + THIN_LINE)
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
            val resource = resourceSetProvider.get.getResource(URI.createFileURI(test.path.toString()),true)
            val context = new StandaloneContext => [
                cancelIndicator = CancelIndicator.NullImpl
                args = test.properties; // FIXME change this
                packageRoot = Paths.get(getRoot()); // FIXME: improve
                hierarchicalBin = true;
            ]
            test.fileConfig = new FileConfig(resource, fileAccess, context);
        } catch (Exception e) {
            test.result = Result.PARSE_FAIL
            restoreOutputs()
            return false
        }
        
        if (test.fileConfig === null || test.fileConfig.resource === null || !test.fileConfig.resource.errors.isEmpty) {
            test.result = Result.PARSE_FAIL
            restoreOutputs()
            return false
        }
        
        // Update the test by applying the configuration. E.g., to carry out an AST transformation.
        if (!configuration.apply(test)) {
            test.result = Result.CONFIG_FAIL
            restoreOutputs()
            return false
        }
        
        if (!this.build) {
            test.properties.setProperty("no-compile", "")
        }
        
        if (!test.fileConfig.resource.allContents.filter(Reactor).exists[it.isMain || it.isFederated]) {
            test.result = Result.NO_MAIN_FAIL
            restoreOutputs()
            return false
        }
        
        // Validate the resource and store issues in the test object.
        try {
            redirectOutputs(test)
            val issues = validator.validate(test.fileConfig.resource, CheckMode.ALL, CancelIndicator.NullImpl)
            if (!issues.isNullOrEmpty) {
                test.issues.append(issues.join(NEW_LINE))
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

    /**
     * Invoke the code generator for the given test.
     * 
     * @return True if code was generated successfully, false otherwise.
     */
    def boolean generateCode(LFTest test) {
        if (test.fileConfig.resource !== null) {
            redirectOutputs(test)
            try {
                generator.generate(test.fileConfig.resource, fileAccess, test.fileConfig.context)
            } catch (Exception e) {
                println(e.stackTrace)
                test.issues.append(e.message)
                test.result = Result.CODE_GEN_FAIL
                restoreOutputs()
                return false
            }
            
            restoreOutputs()
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
            case CPP,
            case CCPP: {
                val bin = test.fileConfig.binPath
                val fullPath = bin.resolve(nameOnly)
                if (Files.exists(fullPath)) {
                    pb = new ProcessBuilder("." + File.separator + nameOnly)
                    pb.directory(bin.toFile)
                } else {
                    if (System.getProperty("os.name").startsWith("Windows")) {
                        val winPath = bin.resolve(nameOnly + ".exe")
                        if (Files.exists(winPath)) {
                            pb = new ProcessBuilder("start", nameOnly + ".exe")
                            pb.directory(bin.toFile)
                        } else {
                            test.issues.append(winPath + ": No such file or directory." + NEW_LINE)
                            test.result = Result.NO_EXEC_FAIL
                        }
                    } else {
                        test.issues.append(fullPath + ": No such file or directory." + NEW_LINE)
                        test.result = Result.NO_EXEC_FAIL
                    }
                }
            }
            case Python: {
                val srcGen = test.fileConfig.getSrcGenPath
                val fullPath = srcGen.resolve(nameOnly + ".py")
                if (Files.exists(fullPath)) {
                    pb = new ProcessBuilder("python3", fullPath.toFile.name)
                    pb.directory(srcGen.toFile)
                } else {
                    test.result = Result.NO_EXEC_FAIL
                    if (pb !== null) {
                        test.issues.append("Process builder: " + pb.toString + NEW_LINE)
                    }
                    if (fullPath !== null) {
                        test.issues.append("File: " + fullPath + NEW_LINE)
                    }
                }
            }
            case TS: {
                val dist = test.fileConfig.getSrcGenPath.resolve("dist")
                val file = dist.resolve(nameOnly + ".js")
                if (Files.exists(file)) {
                    pb = new ProcessBuilder("node", file.toString)
                } else {
                    test.result = Result.NO_EXEC_FAIL
                    if (pb !== null) {
                        test.issues.append("Process builder: " + pb.toString + NEW_LINE)
                    }
                    if (file !== null) {
                        test.issues.append("File: " + file + NEW_LINE)
                    }
                }
            }
        }
        if (pb !== null) {
            try {
                val p = pb.start()
                val stdout = test.exec.recordStdOut(p)
                val stderr = test.exec.recordStdErr(p)
                if(!p.waitFor(MAX_EXECUTION_TIME_SECONDS, TimeUnit.SECONDS)) {
                    stdout.interrupt()
                    stderr.interrupt()
                    p.destroyForcibly()
                    test.result = Result.TEST_TIMEOUT
                } else {
                    if (p.exitValue == 0) {
                        test.result = Result.TEST_PASS
                    } else {
                        test.result = Result.TEST_FAIL
                    }
                }
                
            } catch(Exception e) {
                test.result = Result.TEST_FAIL
            }
            
        } else {
            test.result = Result.NO_EXEC_FAIL
        }

    }

    def compileAndRun(Set<LFTest> tests, Function1<LFTest, Boolean> configuration) {
        val x = 78f / tests.size()
        var marks = 0
        var done = 0
        fileAccess.outputPath = getRoot() + File.separator + FileConfig.DEFAULT_SRC_GEN_DIR
        for (test : tests) {
            if (test.parseAndValidate(configuration) && test.generateCode()) {
                if (run) {
                    test.execute()
                }
            }
            done++
            while (Math.floor(done * x) >= marks && marks < 78) {
                print("=")
                marks++
            }
        }
        while(marks < 78) {
            print("=")
            marks++
        }
        print(NEW_LINE)
    }
}
