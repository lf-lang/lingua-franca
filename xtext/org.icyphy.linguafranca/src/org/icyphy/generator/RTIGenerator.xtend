/* Generator for C RTI. */

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

import static extension org.icyphy.ASTUtils.*
import java.io.File
import java.io.FileOutputStream
import java.util.LinkedHashMap
import java.util.ArrayList
import java.util.List
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.TimeUnit


/**
 * Generator for C RTI.
 * 
 * An instance of this class provides the capabilities
 * to generate C code for an RTI file and to compile it.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 */
class RTIGenerator extends CodeGenerator{
    
    ////////////////////////////////////////////
    //// Private fields.
    
    /**
     * The root filename for the main file containing the source code,
     * without the .lf extension.
     */
    String filename
    
    /**
     * Path to the directory containing the .lf file.
     */
    String directory
    
    /**
     * A list of federate instances.
     */
    List<FederateInstance> federates
    
    /**
     * The threads target parameter, or the default 0 if there is none.
     */
    int targetThreads = 0

    ////////////////////////////////////////////
    //// Public methods.

    new (String filename, String directory, List<FederateInstance> federates,
        int targetThreads
    ) {
        this.filename = filename
        this.directory = directory
        this.federates = federates
        this.targetThreads = targetThreads
    }
    
    // FIXME: Allow target code generators to specify the directory
    // structure for the generated C RTI?
    /** 
     * Create the runtime infrastructure (RTI) source file. This
     * involves generating the code for the RTI, writing it to a file
     * and deleting previously generated output.
     * @param rtiSrcPath The path from the LF file directory 
     * to the directory containing generated
     * RTI C code.
     * @param rtiBinPath The path from the LF file directory
     * to the directory containing the compiled
     * RTI binary.
     * @param targetFast The fast target parameter, or false
     * if there is none.
     * @param federationRTIProperties The federation RTI properties,
     * such as host and port
     */
    def generateFederateRTI(String rtiSrcPath, String rtiBinPath,
        boolean targetFast, LinkedHashMap<String, Object> federationRTIProperties
    ) {
        // Derive target filename from the .lf filename.
        var cFilename = filename + "_RTI.c"

        // Delete source previously produced by the LF compiler.
        var file = new File(directory + rtiSrcPath + File.separator + cFilename)
        if (file.exists) {
            file.delete
        }

        // Delete binary previously produced by the RTI generator.
        file = new File(directory + rtiBinPath + File.separator + filename)
        if (file.exists) {
            file.delete
        }
        
        val rtiCode = new StringBuilder()
        pr(rtiCode, '''
            #ifdef NUMBER_OF_FEDERATES
            #undefine NUMBER_OF_FEDERATES
            #endif
            #define NUMBER_OF_FEDERATES «federates.length»
            #include "rti.c"
            int main(int argc, char* argv[]) {
        ''')
        indent(rtiCode)
        
        // Initialize the array of information that the RTI has about the
        // federates.
        // FIXME: No support below for some federates to be FAST and some REALTIME.
        pr(rtiCode, '''
            for (int i = 0; i < NUMBER_OF_FEDERATES; i++) {
                initialize_federate(i);
                «IF targetFast»
                    federates[i].mode = FAST;
                «ENDIF»
            }
        ''')
        // Initialize the arrays indicating connectivity to upstream and downstream federates.
        for(federate : federates) {
            if (!federate.dependsOn.keySet.isEmpty) {
                // Federate receives non-physical messages from other federates.
                // Initialize the upstream and upstream_delay arrays.
                val numUpstream = federate.dependsOn.keySet.size
                // Allocate memory for the arrays storing the connectivity information.
                pr(rtiCode, '''
                    federates[«federate.id»].upstream = malloc(sizeof(federate_t*) * «numUpstream»);
                    federates[«federate.id»].upstream_delay = malloc(sizeof(interval_t*) * «numUpstream»);
                    federates[«federate.id»].num_upstream = «numUpstream»;
                ''')
                // Next, populate these arrays.
                // Find the minimum delay in the process.
                // FIXME: Zero delay is not really the same as a microstep delay.
                var count = 0;
                for (upstreamFederate : federate.dependsOn.keySet) {
                    pr(rtiCode, '''
                        federates[«federate.id»].upstream[«count»] = «upstreamFederate.id»;
                        federates[«federate.id»].upstream_delay[«count»] = 0LL;
                    ''')
                    // The minimum delay calculation needs to be made in the C code because it
                    // may depend on parameter values.
                    // FIXME: These would have to be top-level parameters, which don't really
                    // have any support yet. Ideally, they could be overridden on the command line.
                    // When that is done, they will need to be in scope here.
                    val delays = federate.dependsOn.get(upstreamFederate)
                    if (delays !== null) {
                        for (value : delays) {
                            pr(rtiCode, '''
                                if (federates[«federate.id»].upstream_delay[«count»] < «value.getRTITime») {
                                    federates[«federate.id»].upstream_delay[«count»] = «value.getRTITime»;
                                }
                            ''')
                        }
                    }
                    count++;
                }
            }
            // Next, set up the downstream array.
            if (!federate.sendsTo.keySet.isEmpty) {
                // Federate sends non-physical messages to other federates.
                // Initialize the downstream array.
                val numDownstream = federate.sendsTo.keySet.size
                // Allocate memory for the array.
                pr(rtiCode, '''
                    federates[«federate.id»].downstream = malloc(sizeof(federate_t*) * «numDownstream»);
                    federates[«federate.id»].num_downstream = «numDownstream»;
                ''')
                // Next, populate the array.
                // Find the minimum delay in the process.
                // FIXME: Zero delay is not really the same as a microstep delay.
                var count = 0;
                for (downstreamFederate : federate.sendsTo.keySet) {
                    pr(rtiCode, '''
                        federates[«federate.id»].downstream[«count»] = «downstreamFederate.id»;
                    ''')
                    count++;
                }
            }
        }
        
        // Start the RTI server before launching the federates because if it
        // fails, e.g. because the port is not available, then we don't want to
        // launch the federates.
        pr(rtiCode, '''
            int socket_descriptor = start_rti_server(«federationRTIProperties.get('port')»);
        ''')
        
        // Generate code that blocks until the federates resign.
        pr(rtiCode, "wait_for_federates(socket_descriptor);")
        
        unindent(rtiCode)
        pr(rtiCode, "}")
        
        var fOut = new FileOutputStream(
                new File(directory + rtiSrcPath + File.separator + cFilename));
        fOut.write(rtiCode.toString().getBytes())
        fOut.close()
    }
    
    /** 
     * Invoke the compiler on the generated RTI
     * @param targetCompiler The compiler target parameter, or null if there is none.
     * @param targetCompilerFlags The compiler flags target parameter, or null if there is none.
     * @param compileAdditionalSources Additional sources to add to the compile command if appropriate.
     * @param compileLibraries Additional libraries to add to the compile command using the "-l" command-line option.
     */
    def compileRTI(String targetCompiler, String targetCompilerFlags,
        ArrayList<String> compileAdditionalSources, ArrayList<String> compileLibraries
    ) {
        var fileToCompile = filename + '_RTI'
        executeCommand(
            compileRTICommand(fileToCompile, targetCompiler, targetCompilerFlags,
                compileAdditionalSources, compileLibraries
        ), directory)
    }
    
    ////////////////////////////////////////////
    //// Private methods.
    
    /** 
     * Return a command to compile the RTI.
     * @param fileToCompile The C filename without the .c extension.
     * @param targetCompiler The compiler target parameter, or null if there is none.
     * @param targetCompilerFlags The compiler flags target parameter, or null if there is none.
     * @param compileAdditionalSources Additional sources to add to the compile command if appropriate.
     * @param compileLibraries Additional libraries to add to the compile command using the "-l" command-line option.
     */
    private def compileRTICommand(String fileToCompile, String targetCompiler,
        String targetCompilerFlags, ArrayList<String> compileAdditionalSources,
        ArrayList<String> compileLibraries
    ) {
        val cFilename = fileToCompile + ".c";            
        val relativeSrcFilename = "src-gen" + File.separator + cFilename;
        val relativeBinFilename = "bin" + File.separator + fileToCompile;

        var compileCommand = newArrayList
        compileCommand.add(targetCompiler)
        val flags = targetCompilerFlags.split(' ')
        compileCommand.addAll(flags)
        compileCommand.add(relativeSrcFilename)
        if (compileAdditionalSources !== null) {
            compileCommand.addAll(compileAdditionalSources)
        }
        if (compileLibraries !== null) {
            compileCommand.addAll(compileLibraries)
        }
        // Only set the output file name if it hasn't already been set
        // using a target property or command line flag.
        if (compileCommand.forall[it.trim != "-o"]) {
            compileCommand.addAll("-o", relativeBinFilename)
        }

        // If threaded computation is requested, add a -pthread option.
        if (targetThreads !== 0) {
            compileCommand.add("-pthread")
        }
        
        // Unlike compileComand in CGenerator, there's no need
        // to potentially add a -c flag here because every federated
        // reactor is a main reactor
        
        return compileCommand
    }
    
    /**
     * Get textual representation of a time in the RTI's target language.
     *
     * @param v A time AST node
     * @return An RTI-compatible (ie. C target) time string
     */
    private def getRTITime(Value v) {  
        var TimeValue time 
        if (v.time !== null) {
            time = new TimeValue(v.time.interval, v.time.unit)
        } else if (v.isZero) {
            time = new TimeValue(0, TimeUnit.NONE)
        } else {
            return v.toText
        }
        if (time.unit != TimeUnit.NONE) {
            return time.unit.name() + '(' + time.time + ')'
        } else {
            return time.time.toString()
        }
    }
    
}