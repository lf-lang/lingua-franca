
import os.path

import runner

lfSourceFilePathBase = '..'
savinaJarPath = '../../../../../../savina/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'
numIterationsDefault = 12
numThreadsDefault = 1

parsers = {
    'savina-akka': 'savina_parser',
    'lf': 'lf_parser'
}

summarizers = {
    'savina-akka': 'default_summarizer',
    'lf': 'default_summarizer'
}

# convenience variables
runPingPongCmd = f'bin/PingPongBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreadsDefault} --count'.split()

experiments = {
    'PingPongProblemSize': {
        1000000: {
            'lf': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(' '),
                   runPingPongCmd + [ '1000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 1000000'.split() ]
        },
        2000000: {
            'lf': [ runPingPongCmd + [ '2000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 2000000'.split() ]
        },
        3000000: {
            'lf': [ runPingPongCmd + [ '3000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 3000000'.split() ]
        },
        4000000: {
            'lf': [ runPingPongCmd + [ '4000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 4000000'.split() ]
        },
        5000000: {
            'lf': [ runPingPongCmd + [ '5000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 5000000'.split() ]
        },
        6000000: {
            'lf': [ runPingPongCmd + [ '6000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 6000000'.split() ]
        },
        7000000: {
            'lf': [ runPingPongCmd + [ '7000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 7000000'.split() ]
        },
        8000000: {
            'lf': [ runPingPongCmd + [ '8000000' ] ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 8000000'.split() ]
        }
    }
}

