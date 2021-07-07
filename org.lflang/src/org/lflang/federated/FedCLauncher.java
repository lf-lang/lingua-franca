package org.lflang.federated;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.generator.FederateInstance;
import org.lflang.generator.c.CCompiler;

/**
 * Utility class that can be used to create a launcher for federated LF programs
 * that are written in C.
 * 
 * @author Soroush Bateni <soroush@utdallas.edu>
 */
public class FedCLauncher extends FedLauncher {

    /**
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter A error reporter for reporting any errors or warnings during the code generation
     */
    public FedCLauncher(
            TargetConfig targetConfig, 
            FileConfig fileConfig,
            ErrorReporter errorReporter
    ) {
        super(targetConfig, fileConfig, errorReporter);
    }
    
    /**
     * Return the compile command for a federate.
     * 
     * @param federate The federate to compile.
     */
    @Override
    protected
    String compileCommandForFederate(FederateInstance federate) {
        CCompiler cCompiler = new CCompiler(targetConfig, fileConfig, federate.getGenerator());
        return String.join(" ", 
                cCompiler.compileCCommand(
                    fileConfig.name+"_"+federate.name, 
                    false, 
                    errorReporter
                ).command());
    }
    
    /**
     * Return the command that will execute a federate, assuming that the current
     * directory is the top-level project folder. This is used to create a launcher script
     * for federates.
     * 
     * @param federate The federate to execute.
     */
    @Override
    protected
    String executeCommandForFederate(FederateInstance federate) {
        return "bin/"+fileConfig.name+"_"+federate.name+" -i '$FEDERATION_ID'";
    }
    

}
