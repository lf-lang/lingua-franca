# This config is intended for a comparison between
# the Lingua Franca C++ implementation with different number of threads
# and the Savina Akka implementation.
#
# The experiment are not supposed to produce line diagrams but
# simple boxes.
#
# These experiment are based on swapping a source file from
# Savina to set the number of threads used by Savina and then
# recompile Savina.
#
# Recommended plotter module: plotterBoxerrorbar

import os

########### INTERFACE ########################################################
# Change parameters listet here according to your environment and likings.
# All paths are relative to the current working directory when running
# this script.
#
# Requirements:
# - Python module cog, install with 'pip3 install cogapp'
# - java version 1.8 for Savina, test with 'java -version'
# - Savina repository from https://github.com/shamsimam/savina
# - Executable lfc in PATH.

# Where are the .lf files of the Savina Cpp implementation?
lfCppSourceFilePathBase = '../../benchmark/Cpp/Savina'

# Path to the original Savina source code modified with cog
# to set the number of worker threads.
savinaSrcPathBase = '../../../savina-original-var-threads'

# Path to the jar file from the Savina benchmark suite:
savinaJarPath = '../../../savina-original-var-threads/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'

# The package path inside of Savina to the benchmarks:
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'

pythonExe = 'python3'
lfcExe = 'lfc'
javaExe = 'java'

# Options for the benchmarks:
numIterationsDefault = 12
numIterationsAkka = 12
########### INTERFACE ########################################################




# predefined global variables for convenience
resourceSavinaAkkaActorOriginal = 'resources/AkkaActor-original.scala'
resourceSavinaAkkaActorThreads = 'resources/AkkaActor-var-threads.scala'
savinaThreadChooserFile = os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")
parsers = {
    'savina-akka': 'parserSavina',
    'lf-cpp': 'parserLfCpp'
}
sequenceColors = {
    'savina-akka': '1',
    'lf-cpp': '2'
}
sequenceNames = {
    'lf-cpp': 'LF',
    'savina-akka': 'AK'
}
arrangementSequences = [
    'savina-akka',
    'lf-cpp'
]
finalCleanup = [
    "rm -rf build include lib share src-gen bin",
    f'cp "{resourceSavinaAkkaActorOriginal}" "{savinaThreadChooserFile}"',
    f'cd {savinaSrcPathBase} && mvn package'
]
initExperiment = [
    f'cp "{resourceSavinaAkkaActorThreads}" "{savinaThreadChooserFile}"'
]
cleanupExperiment = [
    f'cp "{resourceSavinaAkkaActorOriginal}" "{savinaThreadChooserFile}"'
]


# definition of the experiments
experiments = {}


binName = 'ThroughputBenchmarkGenerator'
lfSrcPath = f'fjthrput/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark'
runParamNameLf1 = f'--numMessagesPerReactor'
runParamValue1 = '100000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '60'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf = f'bin/{binName} --fast --numIterations {numIterationsDefault} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1} -a {preParamValue1}'
experiments['ThroughputVarNumWorkers'] = {
    'description': 'Fork join (throughput) benchmark from the Savina suite with a variable number workers',
    'plotTitle': 'Fork join (throughput)',
    'plotXAxisLabel': 'Number of worker threads',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': finalCleanup,
    'initExperiment': initExperiment,
    'cleanupExperiment': cleanupExperiment,
    'sequences': [
        ('1', {
            'lf-cpp': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                       f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                       f'{runCmdLf} --threads 1'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=1 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('2', {
            'lf-cpp': [ f'{runCmdLf} --threads 2'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=2 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('4', {
            'lf-cpp': [ f'{runCmdLf} --threads 4'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=4 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('8', {
            'lf-cpp': [ f'{runCmdLf} --threads 8'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=8 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('16', {
            'lf-cpp': [ f'{runCmdLf} --threads 16'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=16 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        })
    ]
}



binName = 'ApspBenchmarkGenerator'
lfSrcPath = f'apsp/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.apsp.ApspAkkaActorBenchmark'
runParamNameLf1 = f'--maxEdgeWeight'
runParamValue1 = '100'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numNodes'
preParamValue1 = '300'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f'blockSize'
preParamValue2 = '50'
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf = f'bin/{binName} --fast --numIterations {numIterationsDefault} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {preParamValue1} -b {preParamValue2} -w {runParamValue1}'
experiments['Apsp'] = {
    'description': f'All Pairs Shortest Path benchmark from the Savina suite.',
    'plotTitle': f'All Pairs Shortest Path',
    'plotXAxisLabel': 'Number of worker threads',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': finalCleanup,
    'initExperiment': initExperiment,
    'cleanupExperiment': cleanupExperiment,
    'sequences': [
        ('1', {
            'lf-cpp': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                       f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                       f'{runCmdLf} --threads 1'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=1 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('2', {
            'lf-cpp': [ f'{runCmdLf} --threads 2'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=2 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('4', {
            'lf-cpp': [ f'{runCmdLf} --threads 4'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=4 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('8', {
            'lf-cpp': [ f'{runCmdLf} --threads 8'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=8 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('16', {
            'lf-cpp': [ f'{runCmdLf} --threads 16'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=16 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'DictionaryBenchmarkGenerator'
lfSrcPath = f'concdict/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.concdict.DictionaryAkkaActorBenchmark'
runParamNameLf1 = f'--numMessagesPerWorker'
runParamValue1 = '50000'
runParamNameLf2 = f'--writePercentage'
runParamValue2 = '10'
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf = f'bin/{binName} --fast --numIterations {numIterationsDefault} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -e {preParamValue1} -m {runParamValue1} -w {runParamValue2}'
experiments['Dictionary'] = {
    'description': f'Concurrent dictionary benchmark from the Savina suite.',
    'plotTitle': f'Concurrent dictionary',
    'plotXAxisLabel': 'Number of worker threads',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': finalCleanup,
    'initExperiment': initExperiment,
    'cleanupExperiment': cleanupExperiment,
    'sequences': [
        ('1', {
            'lf-cpp': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                       f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                       f'{runCmdLf} --threads 1'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=1 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('2', {
            'lf-cpp': [ f'{runCmdLf} --threads 2'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=2 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('4', {
            'lf-cpp': [ f'{runCmdLf} --threads 4'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=4 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('8', {
            'lf-cpp': [ f'{runCmdLf} --threads 8'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=8 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        }),
        ('16', {
            'lf-cpp': [ f'{runCmdLf} --threads 16'.split() ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=16 {savinaThreadChooserFile}'.split(),
                            f'cd {savinaSrcPathBase} && mvn package',
                            f'{runCmdAkka}'.split() ]
        })
    ]
}









