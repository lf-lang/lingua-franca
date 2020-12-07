import math
import sys

'''Statistical summarizer for measurements.

This accumulator drops the first 5 values and 
calculates the median and the 1st and 3rd quantiles of the rest.
This accumulator could be extended to drop significant
outliners.

Returns (median, firstQuantile, thirdQuantile, arithmeticMean, min, max)

Parameters:
measurements (List): execution times to summarize
parameterValue: runtime parameter used for the measurements
resultFileName (Str): file name that the measurements are taken from; used for error output
'''

numberToDrop = 5

def summarizeMeasurements(measurements, parameterValue, resultFileName):
    
    if len(measurements) == 0:
        print(f'Warning: Summarizer received no values from file {resultFileName}. Check output file and log.', file=sys.stderr)
        return None
    
    # delete the first values as they are assumed to be warm-up
    measurementsCleaned = measurements
    if len(measurements) > numberToDrop:
        measurementsCleaned = measurements[numberToDrop:len(measurements)]
    else:
        #raise "Accumulator expects 3 or more values, make sure to have enough iterations in the experiment."
        print(f'Warning: Summarizer: Too few values to drop {numberToDrop} values from file {resultFileName}. Increase the number of iterations to fix.', file=sys.stderr)
    
    measurementsCleaned.sort()
    
    # min, max
    min = measurementsCleaned[0]
    max = measurementsCleaned[-1]
    
    # arithmetic mean
    arithmeticMean = sum(measurementsCleaned)
    arithmeticMean = int(arithmeticMean / len(measurementsCleaned))
    
    # calculate median
    median = 0
    if len(measurementsCleaned) % 2 == 1:
        median = measurementsCleaned[int(len(measurementsCleaned)/2)]
    else:
        val1 = measurementsCleaned[int(len(measurementsCleaned)/2-1)]
        val2 = measurementsCleaned[int(len(measurementsCleaned)/2)]
        median = int((val1+val2)/2)
    #print("median: "+str(median))
    
    # calculate first quantile
    firstQuantile = 0
    firstQuantileIdx = 0.25 * len(measurementsCleaned)
    if firstQuantileIdx.is_integer():
        tmp = 0.5 * (measurementsCleaned[int(firstQuantileIdx-1)] + measurementsCleaned[int(firstQuantileIdx)])
        firstQuantile = int(tmp)
    else:
        tmp = measurementsCleaned[math.floor(firstQuantileIdx)]
        firstQuantile = int(tmp)
    
    # calculate third quantile
    thirdQuantile = 0
    thirdQuantileIdx = 0.75 * len(measurementsCleaned)
    if thirdQuantileIdx.is_integer():
        
        tmp = 0.5 * (measurementsCleaned[int(thirdQuantileIdx-1)] + measurementsCleaned[int(thirdQuantileIdx)])
        thirdQuantile = int(tmp)
    else:
        tmp = measurementsCleaned[math.floor(thirdQuantileIdx)]
        thirdQuantile = int(tmp)
    
    return (median, firstQuantile, thirdQuantile, arithmeticMean, min, max)
