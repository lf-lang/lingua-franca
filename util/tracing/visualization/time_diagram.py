#!/usr/bin/env python3
import argparse         # For arguments parsing
import pandas as pd     # For csv manipulation
import numpy as np
import os
import sys
from collections import defaultdict
from Reactor import Tag, FOREVER, Enclave


import matplotlib.pyplot as plt

# Define the arguments to pass in the command line
parser = argparse.ArgumentParser(description='Render timing diagrams from enclave traces')
parser.add_argument('enclaveTraces', metavar='Enclave trace files', type=str, nargs='+',
                    help='A list of CSV trace files of enclaves')

# Inconsistency matrix
C = np.array([[0, 50000000], [-1, 0]])

# Availability requirements on each node
A = np.array([-1, 25000000])


def sanitize_args(args):
    for enclaveTrace in args.enclaveTraces:
        if not os.path.isfile(enclaveTrace):
            sys.exit(f"ERROR: Trace file `{enclaveTrace}` does not exist")
        if not enclaveTrace.endswith(".csv"):
            sys.exit(f"ERROR: Trace file `{enclaveTrace} must be converted to CSV")


def load_and_process_csv_file(csv_file):
    # Not implemented yet
    df = pd.read_csv(csv_file)
    # Filter out everything except Reaction Start, Reaction Stop
    df_filtered = df[df[df.columns[0]].str.startswith("Reaction ")]
    
    # Initialize a dictionary to hold start times and a list to hold the new DataFrame data
    start_times = {}
    trace = []

    # Process each row
    for idx, row in df_filtered.iterrows():
        reactorId = row[' Reactor']
        if reactorId.startswith(" enclave_connection_reactor"):
            continue
        reactionId = int(row[' Source'])
        phy = row[' Elapsed Physical Time']
        logTime = row[' Elapsed Logical Time']
        logStep = row[' Microstep']
        event = row['Event']
        # If it's a start event, record the start time
        if event == 'Reaction starts':
            if not reactorId in start_times:
                start_times[reactorId] = [0] * 50
            start_times[reactorId][reactionId] = phy
        # If it's an end event, calculate the execution time and add a row to new_data
        elif event == 'Reaction ends':
            start_time = start_times[reactorId][reactionId]
            assert(start_time > 0)
            execution_time = phy - start_time
            trace.append([f"{reactorId}_{reactionId}",Tag(logTime, logStep),start_time, phy, execution_time, -1,-1])

    # Convert new_data to a DataFrame
    return trace

def get_tag(enclaves, consistency):
    tags = [FOREVER]
    for idx, c in enumerate(consistency):
        if c > 0:
            tags.append(enclaves[idx].get_tag(c))

    return min(tags)

def next_event(enclaves, currentPhy):
    for e in enclaves:
        net = e.next_event_tag()
        tag = get_tag(enclaves, e.consistency)



def check_termination_condition(traces, indices):
    for (trace, idx) in zip(traces, indices):
        if idx < len(trace):
            return False
    return True

def simulate(enclaves):
    indices = [0]*len(enclaves)
    currentTags = [Tag()]*len(enclaves)
    done = False
    traces = [x["trace"] for x in enclaves]

    currentPhy = 0

    while not check_termination_condition(traces, indices):
        next_event(enclaves, indices, currentTags, currentPhy)

args = parser.parse_args()
sanitize_args(args)

enclaves = list()

for idx, enclaveTrace in enumerate(args.enclaveTraces):
    name_w_extension = os.path.basename(enclaveTrace)
    name = os.path.splitext(name_w_extension)[0]
    df = load_and_process_csv_file(enclaveTrace)
    enclave = Enclave(df)
    enclaves.append(enclave)

simulate(enclaves)


trace = enclaves[0].trace
# Plot the timeline
fig, ax = plt.subplots()
xVals = list()
yVals = list()

for i in range(len(df)-1):
    [reactionId, logTag, phyStart, phyEnd, phyDur, phySimStart, phySimEnd] = trace[i]
    log = logTag.time
    phyEndLast = 0
    logEndLast = 0

    if i > 0:
        logEndLast = xVals[-1]
        phyEndLast = yVals[-1]
    if logEndLast < log and phyEndLast<log:
        xVals.append(phyEndLast)
        yVals.append(phyEndLast)
        xVals.append(log)
        yVals.append(log)


    xVals.append(log)
    xVals.append(log)
    yVals.append(phyStart)
    yVals.append(phyEnd)

xMin = 0
xMax = max(xVals)

ax.plot(xVals, yVals)
ax.plot([xMin, xMax], [xMin, xMax])
plt.xlabel('Logical Time')
plt.ylabel('Physical Time (start time of function)')
plt.title('Timeline of Reaction invocations')
plt.show()
