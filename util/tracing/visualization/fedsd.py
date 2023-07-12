'''
Define arrows:
    (x1, y1) ==> (x2, y2), when unique result (this arrow will be tilted)
    (x1, y1) --> (x2, y2), when a possible result (could be not tilted)?
If not arrow, then triangle with text 

In the dataframe, each row will be marked with one op these values:
    - 'arrow': draw a non-dashed arrow
    - 'dot': draw a dot only
    - 'marked': marked, not to be drawn
    - 'pending': pending
    - 'adv': for reporting logical time advancing, draw a simple dash
'''

# Styles to determine appearance:
css_style = ' <style> \
    line { \
        stroke: black; \
        stroke-width: 2; \
    } \
    .ABS {stroke: #d9dd1f; fill: #d9dd1f; } \
    .LTC { stroke: #073b4c; fill: #073b4c;} \
    .T_MSG { stroke: #ef476f; fill: #ef476f} \
    .NET { stroke: #118ab2; fill: #118ab2} \
    .PTAG { stroke: #06d6a0; fill: #06d6a0} \
    .TAG { stroke: #08a578; fill: #08a578} \
    .TIMESTAMP { stroke: grey; fill: grey } \
    .FED_ID {stroke: #80DD99; fill: #80DD99 } \
    .ADV {stroke-linecap="round" ; stroke: "red" ; fill: "red"} \
    text { \
        font-size: smaller; \
        font-family: sans-serif; \
    } \
    text.time {fill: #074936; } \
</style> \
'

#!/usr/bin/env python3
import argparse         # For arguments parsing
import pandas as pd     # For csv manipulation
from os.path import exists
from pathlib import Path
import math
import fedsd_helper as fhlp

# Define the arguments to pass in the command line
parser = argparse.ArgumentParser(description='Set of the csv trace files to render.')
parser.add_argument('-r','--rti', type=str, default="rti.csv",
                    help='RTI csv trace file.')
parser.add_argument('-f','--federates', nargs='+', action='append',
                    help='List of the federates csv trace files.')

# Events matching at the sender and receiver ends depend on whether they are tagged
# (the elapsed logical time and microstep have to be the same) or not. 
# Set of tagged events (messages)
non_tagged_messages = {'FED_ID', 'ACK', 'REJECT', 'ADR_RQ', 'ADR_AD', 'MSG', 'P2P_MSG'}

def load_and_process_csv_file(csv_file) :
    '''
    Loads and processes the csv entries, based on the type of the actor (if RTI
    or federate).

    Args:
     * csv_file: String file name
    Returns:
     * The processed dataframe.
    '''
    # Load tracepoints, rename the columns and clean non useful data
    df = pd.read_csv(csv_file)
    df.columns = ['event', 'reactor', 'self_id', 'partner_id', 'logical_time', 'microstep', 'physical_time', 't', 'ed']
    df = df.drop(columns=['reactor', 't', 'ed'])

    # Remove all the lines that do not contain communication information
    # which boils up to having 'RTI' in the 'event' column
    df = df[df['event'].str.contains('Sending|Receiving|Scheduler advancing time ends') == True]

    # Fix the parameters of the event 'Scheduler advancing time ends'
    # We rely on the fact that the first row of the csv file cannot be the end of advancing time
    id = df.iloc[-1]['self_id']
    df['self_id'] = id
    df = df.astype({'self_id': 'int', 'partner_id': 'int'})

    # Add an inout column to set the arrow direction
    df['inout'] = df['event'].apply(lambda e: 'in' if 'Receiving' in e else 'out')

    # Prune event names
    df['event'] = df['event'].apply(lambda e: fhlp.prune_event_name[e])
    return df


if __name__ == '__main__':
    args = parser.parse_args()

    # The RTI and each of the federates have a fixed x coordinate. They will be
    # saved in a dict
    x_coor = {}
    actors = []
    actors_names = {}
    padding = 50
    spacing = 200       # Spacing between federates

    # Set the RTI x coordinate
    x_coor[-1] = padding * 2
    actors.append(-1)
    actors_names[-1] = "RTI"
   
    trace_df = pd.DataFrame()

    ############################################################################
    #### Federates trace processing
    ############################################################################
    # Loop over the given list of federates trace files 
    if (args.federates) :
        for fed_trace in args.federates[0]:
            if (not exists(fed_trace)):
                print('Warning: Trace file ' + fed_trace + ' does not exist! Will resume though')
                continue
            try:
                fed_df = load_and_process_csv_file(fed_trace)
            except Exception as e:
                print(f"Warning: Problem processing trace file {fed_trace}: `{e}`")
                continue

            if (not fed_df.empty):
                # Get the federate id number
                fed_id = fed_df.iloc[-1]['self_id']
                # Add to the list of sequence diagram actors and add the name
                actors.append(fed_id)
                actors_names[fed_id] = Path(fed_trace).stem
                # Derive the x coordinate of the actor
                x_coor[fed_id] = (padding * 2) + (spacing * (len(actors) - 1))
                fed_df['x1'] = x_coor[fed_id]
                trace_df = pd.concat([trace_df, fed_df])
                fed_df = fed_df[0:0]
    
        
    ############################################################################
    #### RTI trace processing, if any
    ############################################################################
    if (exists(args.rti)):
        rti_df = load_and_process_csv_file(args.rti)
        rti_df['x1'] = x_coor[-1]
    else:
        # If there is no RTI, derive one.
        # This is particularly useful for tracing enclaves
        # FIXME: Currently, `fedsd` is used either for federates OR enclaves.
        # As soon as there is a consensus on how to visualize federations where
        # a federate has several enclves, the utility will be updated.
        rti_df = trace_df[['event', 'self_id', 'partner_id', 'logical_time', 'microstep', 'physical_time', 'inout']].copy()
        rti_df = rti_df[rti_df['event'].str.contains('AdvLT') == False]
        rti_df.columns = ['event', 'partner_id', 'self_id', 'logical_time', 'microstep', 'physical_time', 'inout']
        rti_df['inout'] = rti_df['inout'].apply(lambda e: 'in' if 'out' in e else 'out')
        rti_df['x1'] = rti_df['self_id'].apply(lambda e: x_coor[int(e)])

    trace_df = pd.concat([trace_df, rti_df])

    # Sort all traces by physical time and then reset the index
    trace_df = trace_df.sort_values(by=['physical_time'])
    trace_df = trace_df.reset_index(drop=True)

    # Add the Y column and initialize it with the padding value 
    trace_df['y1'] = math.ceil(padding * 3 / 2) # Or set a small shift

    ############################################################################
    #### Compute the 'y1' coordinates
    ############################################################################
    ppt = 0     # Previous physical time
    cpt = 0     # Current physical time
    py = 0      # Previous y
    min = 15    # Minimum spacing between events when time has not advanced.
    scale = 1   # Will probably be set manually
    first_pass = True
    for index, row in trace_df.iterrows():
        if (not first_pass) :
            cpt = row['physical_time']
            # print('cpt = '+str(cpt)+' and ppt = '+str(ppt))
            # From the email:
            # Y = T_previous + min + log10(1 + (T - T_previous)*scale)
            # But rather think it should be:
            if (cpt != ppt) :
                py = math.ceil(py + min + (1 + math.log10(cpt - ppt) * scale))
            trace_df.at[index, 'y1'] = py

        ppt = row['physical_time']
        py = trace_df.at[index, 'y1']
        first_pass = False

    ############################################################################
    #### Derive arrows that match sided communications
    ############################################################################
    # Intialize all rows as pending to be matched
    trace_df['arrow'] = 'pending'
    trace_df['x2'] = -1
    trace_df['y2'] = -1

    # Iterate and check possible sides
    for index in trace_df.index:
        # If the tracepoint is pending, proceed to look for a match
        if (trace_df.at[index,'arrow'] == 'pending') :
            # Look for a match only if it is not about advancing time
            if (trace_df.at[index,'event'] == 'AdvLT') :
                trace_df.at[index,'arrow'] = 'adv'
                continue
            self_id = trace_df.at[index,'self_id']
            partner_id = trace_df.at[index,'partner_id']
            event =  trace_df.at[index,'event']
            logical_time = trace_df.at[index, 'logical_time']
            microstep = trace_df.at[index, 'microstep']
            inout = trace_df.at[index, 'inout']

            # Match tracepoints
            # Depends on whether the event is tagged or not
            if (trace_df.at[index,'event'] not in non_tagged_messages):
                matching_df = trace_df[\
                    (trace_df['inout'] != inout) & \
                    (trace_df['self_id'] == partner_id) & \
                    (trace_df['partner_id'] == self_id) & \
                    (trace_df['arrow'] == 'pending') & \
                    (trace_df['event'] == event) & \
                    (trace_df['logical_time'] == logical_time) & \
                    (trace_df['microstep'] == microstep) \
                ]
            else :
                matching_df = trace_df[\
                    (trace_df['inout'] != inout) & \
                    (trace_df['self_id'] == partner_id) & \
                    (trace_df['partner_id'] == self_id) & \
                    (trace_df['arrow'] == 'pending') & \
                    (trace_df['event'] == event)
                ]

            if (matching_df.empty) :
                # If no matching receiver, than set the arrow to 'dot',
                # meaning that only a dot will be rendered
                trace_df.at[index, 'arrow'] = 'dot'
            else:
                # If there is one or more matching rows, then consider 
                # the first one
                matching_index = matching_df.index[0]
                matching_row = matching_df.loc[matching_index]
                if (inout == 'out'):
                    trace_df.at[index, 'x2'] = matching_row['x1']
                    trace_df.at[index, 'y2'] = matching_row['y1']
                else:
                    trace_df.at[index, 'x2'] = trace_df.at[index, 'x1'] 
                    trace_df.at[index, 'y2'] = trace_df.at[index, 'y1'] 
                    trace_df.at[index, 'x1'] = matching_row['x1']
                    trace_df.at[index, 'y1'] = matching_row['y1']

                # Mark it, so not to consider it anymore
                trace_df.at[matching_index, 'arrow'] = 'marked'
                trace_df.at[index, 'arrow'] = 'arrow'

    ############################################################################
    #### Write to svg file
    ############################################################################
    svg_width = padding * 2 + (len(actors) - 1) * spacing + padding * 2 + 200
    svg_height = padding + trace_df.iloc[-1]['y1']

    with open('trace_svg.html', 'w', encoding='utf-8') as f:
        # Print header
        f.write('<!DOCTYPE html>\n')
        f.write('<html>\n')
        f.write('<body>\n\n')
        
        f.write('<svg width="'+str(svg_width)+'" height="'+str(svg_height)+'">\n')

        f.write(css_style)
        
        # Print the circles and the names
        for key in x_coor:
            title = actors_names[key]
            if (key == -1):
                f.write(fhlp.svg_string_comment('RTI Actor and line'))
                center = 15
            else:
                f.write(fhlp.svg_string_comment('Federate '+str(key)+': ' + title + ' Actor and line'))
                center = 5
            f.write(fhlp.svg_string_draw_line(x_coor[key], math.ceil(padding/2), x_coor[key], svg_height, False))
            f.write('\t<circle cx="'+str(x_coor[key])+'" cy="'+str(math.ceil(padding/2))+'" r="20" stroke="black" stroke-width="2" fill="white"/>\n')
            f.write('\t<text x="'+str(x_coor[key]-center)+'" y="'+str(math.ceil(padding/2)+5)+'" fill="black">'+title+'</text>\n')

        # Now, we need to iterate over the traces to draw the lines
        f.write(fhlp.svg_string_comment('Draw interactions'))
        for index, row in trace_df.iterrows():
            # For time labels, display them on the left for the RTI, right for everthing else.
            anchor = 'start'
            if (row['self_id'] < 0):
                anchor = 'end'

            # formatted physical time.
            # FIXME: Using microseconds is hardwired here.
            physical_time = f'{int(row["physical_time"]/1000):,}'

            if (row['event'] in {'FED_ID', 'ACK', 'REJECT', 'ADR_RQ', 'ADR_AD', 'MSG', 'P2P_MSG'}):
                label = row['event']
            else:
                label = row['event'] + '(' + f'{int(row["logical_time"]):,}' + ', ' + str(row['microstep']) + ')'
            
            if (row['arrow'] == 'arrow'): 
                f.write(fhlp.svg_string_draw_arrow(row['x1'], row['y1'], row['x2'], row['y2'], label, row['event']))
                if (row['inout'] in 'in'):
                    f.write(fhlp.svg_string_draw_side_label(row['x2'], row['y2'], physical_time, anchor))
                else:
                    f.write(fhlp.svg_string_draw_side_label(row['x1'], row['y1'], physical_time, anchor))
            elif (row['arrow'] == 'dot'):
                if (row['inout'] == 'in'):
                    label = "(in) from " + str(row['partner_id']) + ' ' + label
                else :
                    label = "(out) to " + str(row['partner_id']) + ' ' + label
                
                if (anchor == 'end'):
                    f.write(fhlp.svg_string_draw_side_label(row['x1'], row['y1'], physical_time, anchor))
                    f.write(fhlp.svg_string_draw_dot(row['x1'], row['y1'], label))
                else:
                    f.write(fhlp.svg_string_draw_dot_with_time(row['x1'], row['y1'], physical_time, label))

            elif (row['arrow'] == 'marked'):
                f.write(fhlp.svg_string_draw_side_label(row['x1'], row['y1'], physical_time, anchor))

            elif (row['arrow'] == 'adv'):
                f.write(fhlp.svg_string_draw_adv(row['x1'], row['y1'], label))

        f.write('\n</svg>\n\n')

        # Print footer
        f.write('</body>\n')
        f.write('</html>\n')

    # Write to a csv file, just to double check
    trace_df.to_csv('all.csv', index=True)