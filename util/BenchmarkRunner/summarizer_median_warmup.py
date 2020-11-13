

def summarizeMeasurements(measurements, parameterValue, resultFileName):
    '''This accumulator drops the first 12 values and 
    calculates the median of the rest.
    This accumulator could be extended to drop significant
    outliners.
    
    Parameters:
    measurements (list): List of numbers that is created by a parser from reading the output.
    parameterValue (str): A string that can be cast to a number to use it in the calculations, for example to calculate a
    resultFileName (str): Name of the file the 'measurements' are parsed from. Used only for debugging.
    '''
    if len(measurements) == 0:
        print(f'Warning: Summarizer received no values from file {resultFileName}. Returning summary value 0, check output file and log.')
        return 0
    
    # delete the first 3 values as they are assumed to be warm-up
    measurementsCleaned = measurements
    if len(measurements) > 3:
        measurementsCleaned = measurements[3:len(measurements)]
    else:
        #raise "Accumulator expects 3 or more values, make sure to have enough iterations in the experiment."
        print('Warning: Summarizer "default_summarizer" found not more than 3 iterations.')
    
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
