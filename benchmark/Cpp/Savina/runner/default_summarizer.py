

def summarizeMeasurements(measurements):
    '''This accumulator drops the first 12 values and 
    calculates the median of the rest.
    This accumulator could be extended to drop significant
    outliners.'''
    
    if len(measurements) < 3:
        raise "Accumulator expects 3 or more values, make sure to have enough iterations in the experiment."
    
    measurementsCleaned = measurements[3:len(measurements)]
    
    # calculate median
    median = 0
    if len(measurementsCleaned) % 2 == 1:
        median = measurementsCleaned[int(len(measurementsCleaned)/2)]
    else:
        val1 = measurementsCleaned[int(len(measurementsCleaned)/2-1)]
        val2 = measurementsCleaned[int(len(measurementsCleaned)/2)]
        median = int((val1+val2)/2)
    #print("median: "+str(median))
    return median