import re

def parse(inputFile):
    
    results = []
    for line in inputFile:
        pattern = r'Iteration\s*[0-9]*\s*\-\s*([\.0-9]*)\s*ms'
        measurement = re.findall(pattern, line)
        if len(measurement) == 1:
            numMillis = re.findall(r'([0-9]*)', measurement[0])
            results.append(int(numMillis[0]))
    #print(results)
    return results