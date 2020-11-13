import re

def parse(inputFile, resultFileName):
    
    results = []
    for line in inputFile:
        pattern = r'Iteration\-[0-9]*\:\s* ([\.0-9]*) \s*ms'
        measurement = re.findall(pattern, line)
        if len(measurement) == 1:
            numMillis = re.findall(r'([0-9]*)', measurement[0])
            results.append(int(numMillis[0]))
    #print(results)
    return results