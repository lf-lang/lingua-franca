# This config is runs the concurrency benchmarks from Savina.
#
# Each experiment only has one sequence parameter, they are
# intended to produce box plots and not line diagrams.
#
# It runs the Akka implementation and LF C++ implementations.
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

# Path to the jar file from the Savina benchmark suite:
savinaJarPath = '../../../savina-original-var-threads/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'

# The package path inside of Savina to the benchmarks:
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'

# Executables to use
pythonExe = 'python3'
lfcExe = 'lfc'
javaExe = 'java'

# Options for the benchmarks:
numIterationsDefault = 12
numIterationsAkka = 12
########### INTERFACE ########################################################





# predefined global variables for convenience
parsers = {
    'savina-akka-default': 'parserSavina',
    'lf-cpp-1': 'parserLfCpp',
    'lf-cpp-2': 'parserLfCpp',
    'lf-cpp-4': 'parserLfCpp',
    'lf-cpp-8': 'parserLfCpp',
    'lf-cpp-16': 'parserLfCpp',
    'lf-c-1': 'parserLfC'
}
sequenceColors = {
    'savina-akka-default': '1',
    'lf-cpp-1': '2',
    'lf-cpp-2': '3',
    'lf-cpp-4': '4',
    'lf-cpp-8': '5',
    'lf-cpp-16': '6',
    'lf-c-1': '7'
}
sequenceNames = {
    'lf-cpp-1': 'LF1',
    'lf-cpp-2': 'LF2',
    'lf-cpp-4': 'LF4',
    'lf-cpp-8': 'LF8',
    'lf-cpp-16': 'LF16',
    'savina-akka-default': 'AK',
    'lf-c-1': 'LF C (1 thread)'
}
arrangementSequences = [
    'savina-akka-default',
    'lf-cpp-1',
    'lf-cpp-2',
    'lf-cpp-4',
    'lf-cpp-8',
    'lf-cpp-16',
    'lf-c-1',
]
cleanup = [
    "rm -rf build include lib share src-gen bin"
]

# parameters for the different benchmarks to run
numThreadsLfCpp1 = 1
numThreadsLfCpp2 = 2
numThreadsLfCpp4 = 4
numThreadsLfCpp8 = 8
numThreadsLfCpp16 = 16


experiments = {}


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
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -e {preParamValue1} -m {runParamValue1} -w {runParamValue2}'
experiments['Dictionary'] = {
    'description': f'Concurrent dictionary benchmark from the Savina suite.',
    'plotTitle': f'Concurrent dictionary',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'SortedListBenchmarkGenerator'
lfSrcPath = f'concsll/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.concsll.SortedListAkkaActorBenchmark'
runParamNameLf1 = f'--numMessagesPerWorker'
runParamValue1 = '10000'
runParamNameLf2 = f'--writePercentage'
runParamValue2 = '10'
runParamNameLf3 = f'--sizePercentage'
runParamValue3 = '1'
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -e {preParamValue1} -m {runParamValue1} -w {runParamValue2} -s {runParamValue3}'
experiments['SortedLinkedList'] = {
    'description': f'Sorted linked list benchmark from the Savina suite.',
    'plotTitle': f'Sorted linked list',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'ProdConsBenchmarkGenerator'
lfSrcPath = f'bndbuffer/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.bndbuffer.ProdConsAkkaActorBenchmark'
runParamNameLf1 = f'--bufferSize'
runParamValue1 = '50'
runParamNameLf2 = f'--prodCost'
runParamValue2 = '25'
runParamNameLf3 = f'--consCost'
runParamValue3 = '25'
runParamNameLf4 = f'--numItemsPerProducer'
runParamValue4 = '2000'
preParamNameLf1 = f'numProducers'
preParamValue1 = '40'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f'numConsumers'
preParamValue2 = '40'
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -bb {runParamValue1} -np {preParamValue1} -nc {preParamValue2} -pc {runParamValue2} -cc {runParamValue3} -ipp {runParamValue4} -nm 1'
experiments['ProducerConsumer'] = {
    'description': f'Producer-Consumer with Bounded Buffer benchmark from the Savina suite.',
    'plotTitle': f'Producer-Consumer',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}



binName = 'PhilosopherBenchmarkGenerator'
lfSrcPath = f'philosopher/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.philosopher.PhilosopherAkkaActorBenchmark'
runParamNameLf1 = f'--numEatingRounds'
runParamValue1 = '25000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numPhilosophers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {preParamValue1} -m {runParamValue1} -c 1'
experiments['DiningPhilosophers'] = {
    'description': f'Dining Philosophers benchmark from the Savina suite.',
    'plotTitle': f'Dining Philosophers',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}



binName = 'SleepingBarberBenchmarkGenerator'
lfSrcPath = f'barber/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.barber.SleepingBarberAkkaActorBenchmark'
runParamNameLf1 = f'--waitingRoomSize'
runParamValue1 = '1000'
runParamNameLf2 = f'--averageProductionRate'
runParamValue2 = '1000'
runParamNameLf3 = f'--averageHaircutRate'
runParamValue3 = '1000'
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numHaircuts'
preParamValue1 = '2000'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {preParamValue1} -w {runParamValue1} -pr {runParamValue2} -hr {runParamValue3}'
experiments['SleepingBarber'] = {
    'description': f'Sleeping Barber benchmark from the Savina suite.',
    'plotTitle': f'Sleeping Barber',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'CigaretteSmokerBenchmarkGenerator'
lfSrcPath = f'cigsmok/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.cigsmok.CigaretteSmokerAkkaActorBenchmark'
runParamNameLf1 = f'--numRounds'
runParamValue1 = '1500'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numSmokers'
preParamValue1 = '200'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -r {runParamValue1} -s {preParamValue1}'
experiments['CigaretteSmokers'] = {
    'description': f'Cigarette Smokers benchmark from the Savina suite.',
    'plotTitle': f'Cigarette Smokers',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'LogisticMapBenchmarkGenerator'
lfSrcPath = f'logmap/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.logmap.LogisticMapAkkaManualStashActorBenchmark'
runParamNameLf1 = f'--numTerms'
runParamValue1 = '25000'
runParamNameLf2 = f'--startRate'
runParamValue2 = '3.46'
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numSeries'
preParamValue1 = '10'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -t {runParamValue1} -s {preParamValue1} -r {runParamValue2}'
experiments['LogisticMap'] = {
    'description': f'Logistic Map Series benchmark from the Savina suite.',
    'plotTitle': f'Logistic Map Series',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'BankingBenchmarkGenerator'
lfSrcPath = f'banking/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.banking.BankingAkkaManualStashActorBenchmark'
runParamNameLf1 = f'--numTransactions'
runParamValue1 = '10000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numAccounts'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -a {preParamValue1} -n {runParamValue1}'
experiments['BankingTransaction'] = {
    'description': f'Bank Transaction benchmark from the Savina suite.',
    'plotTitle': f'Bank Transaction',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}
