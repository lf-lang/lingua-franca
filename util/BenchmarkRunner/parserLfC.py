import re

def parse(inputFile, resultFileName):
    
    results = []
    for line in inputFile:
        pattern = r'physical time \(in nsec\)\:\s*([\,0-9]*)\s*$'
        measurement = re.findall(pattern, line)
        if len(measurement) == 1:
            #numMillis = re.findall(r'([0-9]*)\,', measurement[0])
            positions = measurement[0].split(',')
            # remove last 2 entries: nsec, micro-sec
            if len(positions) >= 2:
                positions = positions[:len(positions)-2]
            print(positions)
            numMillis = 0
            if len(positions) >= 1:
                numMillis = ''.join(positions)
            print(numMillis)
            results.append(int(numMillis))
    print(results)
    return results
