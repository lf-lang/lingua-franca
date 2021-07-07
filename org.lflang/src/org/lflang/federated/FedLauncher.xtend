package org.lflang.federated

import java.io.FileOutputStream
import java.util.ArrayList
import java.util.LinkedHashMap
import java.util.List
import org.lflang.FileConfig
import org.lflang.TargetConfig
import org.lflang.TargetProperty.ClockSyncMode
import org.lflang.generator.FederateInstance
import org.lflang.generator.GeneratorBase
import org.lflang.generator.c.CCompiler

class FedLauncher {
    
    var TargetConfig targetConfig;
    var FileConfig fileConfig;
    var String topLevelName;
    var GeneratorBase generator;
    
    /**
     * FIXME
     */
    new (
        TargetConfig targetConfig,
        FileConfig fileConfig,
        String topLevelName,
        GeneratorBase generator
    ) {
        this.targetConfig = targetConfig;
        this.fileConfig = fileConfig;
        this.topLevelName = topLevelName;
        this.generator = generator;
    }
    
    /**
     * Return the compile command for a federate.
     * 
     * @param federate The federate to compile.
     */
    def String compileCommandForFederate(FederateInstance federate) {
        val cCompiler = new CCompiler(targetConfig, fileConfig, generator);
        return cCompiler.compileCCommand(
                    '''«topLevelName»_«federate.name»''', 
                    false, 
                    generator.errorReporter
                ).command.join(" ")
    }
    
    /**
     * Return the command that will execute a federate, assuming that the current
     * directory is the top-level project folder. This is used to create a launcher script
     * for federates.
     * 
     * @param federate The federate to execute.
     */
    def String executeCommandForFederate(FederateInstance federate) {
        return "bin/"+topLevelName+"_"+federate.name+" -i '$FEDERATION_ID'";
    }
    
    /**
     * Create the launcher shell scripts. This will create one or two files
     * in the output path (bin directory). The first has name equal to
     * the filename of the source file without the ".lf" extension.
     * This will be a shell script that launches the
     * RTI and the federates.  If, in addition, either the RTI or any
     * federate is mapped to a particular machine (anything other than
     * the default "localhost" or "0.0.0.0"), then this will generate
     * a shell script in the bin directory with name filename_distribute.sh
     * that copies the relevant source files to the remote host and compiles
     * them so that they are ready to execute using the launcher.
     * 
     * A precondition for this to work is that the user invoking this
     * code generator can log into the remote host without supplying
     * a password. Specifically, you have to have installed your
     * public key (typically found in ~/.ssh/id_rsa.pub) in
     * ~/.ssh/authorized_keys on the remote host. In addition, the
     * remote host must be running an ssh service.
     * On an Arch Linux system using systemd, for example, this means
     * running:
     * 
     *     sudo systemctl <start|enable> ssh.service
     * 
     * Enable means to always start the service at startup, whereas
     * start means to just start it this once.
     * 
     * On MacOS, open System Preferences from the Apple menu and 
     * click on the "Sharing" preference panel. Select the checkbox
     * next to "Remote Login" to enable it.
     * 
     * In addition, every host must have OpenSSL installed, with at least
     * version 1.1.1a.  You can check the version with
     * 
     *     openssl version
     * 
     * @param coreFiles The files from the core directory that must be
     *  copied to the remote machines.
     * @param federates A list of federate instances in the federation
     * @param federationRTIProperties Contains relevant properties of the RTI.
     *  Can have values for 'host', 'dir', and 'user' 
     * 
     */
    def createLauncher(
        ArrayList<String> coreFiles,
        List<FederateInstance> federates,
        LinkedHashMap<String, Object> federationRTIProperties
        
    ) {
        // NOTE: It might be good to use screen when invoking the RTI
        // or federates remotely so you can detach and the process keeps running.
        // However, I was unable to get it working properly.
        // What this means is that the shell that invokes the launcher
        // needs to remain live for the duration of the federation.
        // If that shell is killed, the federation will die.
        // Hence, it is reasonable to launch the federation on a
        // machine that participates in the federation, for example,
        // on the machine that runs the RTI.  The command I tried
        // to get screen to work looks like this:
        // ssh -t «target» cd «path»; screen -S «filename»_«federate.name» -L bin/«filename»_«federate.name» 2>&1
        
        //var outPath = binGenPath

        val shCode = new StringBuilder()
        val distCode = new StringBuilder()
        shCode.append('''
            #!/bin/bash
            # Launcher for federated «topLevelName».lf Lingua Franca program.
            # Uncomment to specify to behave as close as possible to the POSIX standard.
            # set -o posix
            
            # Enable job control
            set -m
            shopt -s huponexit
            
            # Set a trap to kill all background jobs on error or control-C
            # Use two distinct traps so we can see which signal causes this.
            cleanup() {
                printf "Killing federate %s.\n" ${pids[*]}
                # The || true clause means this is not an error if kill fails.
                kill ${pids[@]} || true
                printf "#### Killing RTI %s.\n" ${RTI}
                kill ${RTI} || true
                exit 1
            }
            cleanup_err() {
                echo "#### Received ERR signal on line $1. Invoking cleanup()."
                cleanup
            }
            cleanup_sigint() {
                echo "#### Received SIGINT signal on line $1. Invoking cleanup()."
                cleanup
            }
            
            trap 'cleanup_err $LINENO' ERR
            trap 'cleanup_sigint $LINENO' SIGINT

            # Create a random 48-byte text ID for this federation.
            # The likelihood of two federations having the same ID is 1/16,777,216 (1/2^24).
            FEDERATION_ID=`openssl rand -hex 24`
            echo "Federate «topLevelName» in Federation ID '$FEDERATION_ID'"
            # Launch the federates:
        ''')
        val distHeader = '''
            #!/bin/bash
            # Distributor for federated «topLevelName».lf Lingua Franca program.
            # Uncomment to specify to behave as close as possible to the POSIX standard.
            # set -o posix
        '''
        val host = federationRTIProperties.get('host')
        var target = host

        var path = federationRTIProperties.get('dir')
        if(path === null) path = '''LinguaFrancaRemote'''

        var user = federationRTIProperties.get('user')
        if (user !== null) {
            target = user + '@' + host
        }
        
        var RTILaunchString = '''
        RTI -i ${FEDERATION_ID} \
                         -n «federates.size» \
                         -c «targetConfig.clockSync.toString()» «IF targetConfig.clockSync == ClockSyncMode.ON» \
                          period «targetConfig.clockSyncOptions.period.toNanoSeconds» «ENDIF» \
                          exchanges-per-interval «targetConfig.clockSyncOptions.trials» \
                          &
        '''
        
        // Launch the RTI in the foreground.
        if (host == 'localhost' || host == '0.0.0.0') {
            // FIXME: the paths below will not work on Windows
            shCode.append( '''
                echo "#### Launching the runtime infrastructure (RTI)."
                # First, check if the RTI is on the PATH
                if ! command -v RTI &> /dev/null
                then
                    echo "RTI could not be found."
                    echo "The source code can be found in org.lflang/src/lib/core/federated/RTI"
                    exit
                fi                
                # The RTI is started first to allow proper boot-up
                # before federates will try to connect.
                # The RTI will be brought back to foreground
                # to be responsive to user inputs after all federates
                # are launched.
                «RTILaunchString»
                # Store the PID of the RTI
                RTI=$!
                # Wait for the RTI to boot up before
                # starting federates (this could be done by waiting for a specific output
                # from the RTI, but here we use sleep)
                sleep 1
            ''')
        } else {
            // Start the RTI on the remote machine.
            // FIXME: Should $FEDERATION_ID be used to ensure unique directories, executables, on the remote host?
            // Copy the source code onto the remote machine and compile it there.
            if (distCode.length === 0) distCode.append(distHeader+"\n");
            
            val logFileName = '''log/«topLevelName»_RTI.log'''

            // Launch the RTI on the remote machine using ssh and screen.
            // The -t argument to ssh creates a virtual terminal, which is needed by screen.
            // The -S gives the session a name.
            // The -L option turns on logging. Unfortunately, the -Logfile giving the log file name
            // is not standardized in screen. Logs go to screenlog.0 (or screenlog.n).
            // FIXME: Remote errors are not reported back via ssh from screen.
            // How to get them back to the local machine?
            // Perhaps use -c and generate a screen command file to control the logfile name,
            // but screen apparently doesn't write anything to the log file!
            //
            // The cryptic 2>&1 reroutes stderr to stdout so that both are returned.
            // The sleep at the end prevents screen from exiting before outgoing messages from
            // the federate have had time to go out to the RTI through the socket.
            RTILaunchString = '''
                RTI -i '${FEDERATION_ID}' \
                                 -n «federates.size» \
                                 -c «targetConfig.clockSync.toString()» «IF targetConfig.clockSync == ClockSyncMode.ON» \
                                  period «targetConfig.clockSyncOptions.period.toNanoSeconds» «ENDIF» \
                                  exchanges-per-interval «targetConfig.clockSyncOptions.trials» \
                                  &
            '''
            
            shCode.append( '''
                echo "#### Launching the runtime infrastructure (RTI) on remote host «host»."
                # FIXME: Killing this ssh does not kill the remote process.
                # A double -t -t option to ssh forces creation of a virtual terminal, which
                # fixes the problem, but then the ssh command does not execute. The remote
                # federate does not start!
                ssh «target» 'mkdir -p log; \
                    echo "-------------- Federation ID: "'$FEDERATION_ID' >> «logFileName»; \
                    date >> «logFileName»; \
                    echo "Executing RTI: «RTILaunchString»" 2>&1 | tee -a «logFileName»; \
                    # First, check if the RTI is on the PATH
                    if ! command -v RTI &> /dev/null
                    then
                        echo "RTI could not be found."
                        echo "The source code can be found in org.lflang/src/lib/core/federated/RTI"
                        exit
                    fi
                    «RTILaunchString» 2>&1 | tee -a «logFileName»' &
                # Store the PID of the channel to RTI
                RTI=$!
                # Wait for the RTI to boot up before
                # starting federates (this could be done by waiting for a specific output
                # from the RTI, but here we use sleep)
                sleep 1
            ''')
        }
                
        // Index used for storing pids of federates
        var federateIndex = 0
        for (federate : federates) {
            if (federate.host !== null && federate.host != 'localhost' && federate.host != '0.0.0.0') {
                if(distCode.length === 0) distCode.append(distHeader+"\n");
                val logFileName = '''log/«topLevelName»_«federate.name».log'''
                val compileCommand = compileCommandForFederate(federate);
                //'''«targetConfig.compiler» src-gen/«topLevelName»_«federate.name».c -o bin/«topLevelName»_«federate.name» -pthread «targetConfig.compilerFlags.join(" ")»'''
                // FIXME: Should $FEDERATION_ID be used to ensure unique directories, executables, on the remote host?
                shCode.append( '''
                    echo "Making directory «path» and subdirectories src-gen, bin, and log on host «federate.host»"
                    # The >> syntax appends stdout to a file. The 2>&1 appends stderr to the same file.
                    ssh «federate.host» '\
                        mkdir -p «path»/src-gen/federated/«topLevelName»/core «path»/bin «path»/log «path»/src-gen/core/federated; \
                        echo "--------------" >> «path»/«logFileName»; \
                        date >> «path»/«logFileName»;
                    '
                    pushd «fileConfig.srcGenPath» > /dev/null
                    echo "Copying LF core files to host «federate.host»"
                    scp -r core «federate.host»:«path»/src-gen/federated/«topLevelName»
                    echo "Copying source files to host «federate.host»"
                    scp «topLevelName»_«federate.name».c «FOR file:targetConfig.filesNamesWithoutPath SEPARATOR " "»«file»«ENDFOR» ctarget.h «federate.host»:«path»/src-gen/federated/«topLevelName»
                    popd > /dev/null
                    echo "Compiling on host «federate.host» using: «compileCommand»"
                    ssh «federate.host» 'cd «path»; \
                        echo "In «path» compiling with: «compileCommand»" >> «logFileName» 2>&1; \
                        # Capture the output in the log file and stdout. \
                        «compileCommand» 2>&1 | tee -a «logFileName»;'
                ''')
                val executeCommand = executeCommandForFederate(federate);
                shCode.append( '''
                    echo "#### Launching the federate «federate.name» on host «federate.host»"
                    # FIXME: Killing this ssh does not kill the remote process.
                    # A double -t -t option to ssh forces creation of a virtual terminal, which
                    # fixes the problem, but then the ssh command does not execute. The remote
                    # federate does not start!
                    ssh «federate.host» '\
                        cd «path»; \
                        echo "-------------- Federation ID: "'$FEDERATION_ID' >> «logFileName»; \
                        date >> «logFileName»; \
                        echo "In «path», executing: «executeCommand»" 2>&1 | tee -a «logFileName»; \
                        «executeCommand» 2>&1 | tee -a «logFileName»' &
                    pids[«federateIndex++»]=$!
                ''')                
            } else {
                shCode.append( '''
                    echo "#### Launching the federate «federate.name»."
                    «fileConfig.binPath.resolve(topLevelName)»_«federate.name» -i $FEDERATION_ID &
                    pids[«federateIndex++»]=$!
                ''')                
            }
        }
        if (host == 'localhost' || host == '0.0.0.0') {
            // Local PID managements
            shCode.append( '''
                echo "#### Bringing the RTI back to foreground so it can receive Control-C."
                fg %1
            ''')
        }
        // Wait for launched processes to finish
        shCode.append( '''
            echo "RTI has exited. Wait for federates to exit."
            # Wait for launched processes to finish.
            # The errors are handled separately via trap.
            for pid in "${pids[@]}"
            do
                wait $pid
            done
            echo "All done."
        ''')

        // Write the launcher file.
        // Delete file previously produced, if any.
        var file = fileConfig.binPath.resolve(topLevelName).toFile
        if (file.exists) {
            file.delete
        }
                
        var fOut = new FileOutputStream(file)
        fOut.write(shCode.toString().getBytes())
        fOut.close()
        if (!file.setExecutable(true, false)) {
            generator.errorReporter.reportWarning("Unable to make launcher script executable.")
        }
        
        // Write the distributor file.
        // Delete the file even if it does not get generated.
        file = fileConfig.binPath.resolve(topLevelName + '_distribute.sh').toFile
        if (file.exists) {
            file.delete
        }
        if (distCode.length > 0) {
            fOut = new FileOutputStream(file)
            fOut.write(distCode.toString().getBytes())
            fOut.close()
            if (!file.setExecutable(true, false)) {
                generator.errorReporter.reportWarning("Unable to make distributor script executable.")
            }
        }
    }
}