'''
Define arrows:
    (x1, y1) ==> (x2, y2), when unique result (this arrow will be tilted)
    (x1, y1) --> (x2, y2), when a possible result (could be not tilted)?
If not arrow, then triangle with text 

In the dataframe, each arrow will be marked as:
    - 'arrow': draw a non-dashed arrow
    - 'dashedarrow': draw dashed arrow
    - 'dot': draw the triangle only
    - 'marked': marked, not to be drawn
    - 'pending': pending
'''


#!/usr/bin/env python3
import argparse         # For arguments parsing
import pandas as pd     # For csv manipulation
from os.path import exists
import math
import fedsd_helper as fhlp

# Define the arguments to pass in the command line
parser = argparse.ArgumentParser(description='Set of the csv trace files to render.')
parser.add_argument('-r','--rti', type=str, default="rti.csv",
                    help='RTI csv trace file.')
parser.add_argument('-f','--federates', nargs='+', action='append',
                    help='List of the federates csv trace files.')


''' Clock synchronization error '''
''' FIXME: There should be a value for each communicating pair '''
clock_sync_error = 0

''' Bound on the network latency '''
''' FIXME: There should be a value for each communicating pair '''
network_latency = 250000 # That is 100us


def load_and_process_csv_file(csv_file, rti) :
    '''
    Loads and processes the csv entries, based on the type of the actor (if RTI
    or federate).

    Args:
     * csv_file: String file name
     * rti: Bool True if it the RTI, False otherwise
    Returns:
     * The processed dataframe.
    '''
    # Load RTI tracepoints, rename the columns and clean non useful data
    df = pd.read_csv(csv_file)
    print
    if (rti == True):
        df.columns = ['event', 'r', 'partner_id', 'w', 'logical_time', 'm', 'physical_time', 't', 'ed']
        # Set that these are the RTI information
        df['self_id'] = -1
        # df['partner_id'] = int(df['partner_id'])
    else:
        df.columns = ['event', 'r', 'self_id', 'w', 'logical_time', 'm', 'physical_time', 't', 'ed']
        # Set that these are the RTI information
        # FIXME: Here, we assume that the coordination in centralized. 
        # To be updated for the decentralized case...
        df['partner_id'] = -1
        # df['self_id'] = int(df['partner_id'])
    
    # Remove non-needed information
    df = df.drop(columns=['r', 'w', 'm', 't', 'ed'])

    # Remove all the lines that do not contain communication information
    # which boils up to having 'RTI' in the 'event' column
    df = df[df['event'].str.contains('RTI') == True]
    df = df.astype({'self_id': 'int', 'partner_id': 'int'})

    # Add an inout column to set the arrow direction
    df['inout'] = df['event'].apply(lambda e: 'in' if 'receives' in e else 'out')

    # Prune event names
    df['event'] = df['event'].apply(lambda e: fhlp.prune_event_name[e])
    return df


if __name__ == '__main__':
    args = parser.parse_args()

    # Check if the RTI trace file exists
    if (not exists(args.rti)):
        print('Error: No RTI csv tarce file!')
        # FIXME: Exit?
        exit(0)
    
    # The RTI and each of the federates have a fixed x coordinate. They will be
    # saved in a dict
    x_coor = {}
    actors = []
    padding = 50
    spacing = 200       # Spacing between actors
    
    ############################################################################
    #### RTI trace processing
    ############################################################################
    trace_df = load_and_process_csv_file(args.rti, True)
    x_coor[-1] = padding
    actors.append(-1)
    # Temporary use
    trace_df['x1'] = x_coor[-1]

    ############################################################################
    #### Federates trace processing
    ############################################################################
    # Loop over the given list of federates trace files 
    if (args.federates) :
        for fed_trace in args.federates[0]:
            if (not exists(fed_trace)):
                print('Warning: Trace file ' + fed_trace + ' does not exist! Will resume though')
                continue
            fed_df = load_and_process_csv_file(fed_trace, False)
            if (not fed_df.empty):
                # Get the federate id number
                fed_id = fed_df.iloc[-1]['self_id']
                # Add to the list of sequence diagram actors 
                actors.append(fed_id)
                # Derive the x coordinate of the actor
                x_coor[fed_id] = padding + (spacing * (len(actors)-1))
                fed_df['x1'] = x_coor[fed_id]
                # Append into trace_df
                trace_df = trace_df.append(fed_df, sort=False, ignore_index=True)
                fed_df = fed_df[0:0]
    
    # Sort all traces by physical time and then reset the index
    trace_df = trace_df.sort_values(by=['physical_time'])
    trace_df = trace_df.reset_index(drop=True)

    # FIXME: For now, we need to remove the rows with negative physical time values...
    # Until the reason behinf such values is investigated. The negative physical
    # time is when federates are still in the process of joining
    trace_df = trace_df[trace_df['physical_time'] >= 0]

    # Add the Y column and initialize it with the padding value 
    trace_df['y1'] = math.ceil(padding * 3 / 2) # Or set a small shift

    ############################################################################
    #### Compute the 'y1' coordinates
    ############################################################################
    ppt = 0     # Previous physical time
    cpt = 0     # Current physical time
    py = 0      # Previous y
    min = 10    # Will probably be set manually
    scale = 1   # Will probably be set manually
    first_pass = True
    for index, row in trace_df.iterrows():
        if (not first_pass) :
            cpt = int(row['physical_time'])
            # print('cpt = '+str(cpt)+' and ppt = '+ppt)
            # From the email:
            # Y = T_previous + min + log10(1 + (T - T_previous)*scale)
            # But rather think it should be:
            py = math.ceil(py + min + (1 + math.log10(cpt - ppt) * scale))
            trace_df.at[index, 'y1'] = py

        ppt = int(row['physical_time'])
        py = trace_df.at[index, 'y1']
        first_pass = False

    ############################################################################
    #### Derive arrows that match sided communications
    ############################################################################
    # Intialize all rows as pending to be matched
    trace_df['arrow'] = 'pending'
    trace_df['x2'] = -1
    trace_df['y2'] = -1

    # Because pandas library prevents writing the dataframe when iterating, but 
    # the row at the cueent index, the turnaround is to save the indexes to be
    # modified and then check within the iterations
    indexes_to_mark = []
    # Iterate and check possible sides
    for index, row in trace_df.iterrows():
        # Check is the index is to be marked with 'marked'
        if (index in indexes_to_mark):
            trace_df.at[index, 'arrow'] = 'marked'
            continue
        
        # If not, and if it is a pending tracepoint, proceed to look for a match
        if (row['arrow'] == 'pending') :
            physical_time = row['physical_time']
            self_id = int(row['self_id'])
            partner_id = int(row['partner_id'])
            event = row['event']

            # Depending on the direction, compute the possible time interval
            # and choose the row 
            if ('out' in row['inout']):
                # Compute the possible timestamps interval at the receiver side
                physical_time_start = physical_time - clock_sync_error
                physical_time_end = physical_time + clock_sync_error + network_latency
                
                # Match with 'in' tracepoints
                matching_df = trace_df[\
                    (trace_df['physical_time'] >= physical_time_start) & \
                    (trace_df['physical_time'] <= physical_time_end) & \
                    (trace_df['inout'] == 'in') & \
                    (trace_df['self_id'] == partner_id) & \
                    (trace_df['partner_id'] == self_id) & \
                    (trace_df['arrow'] == 'pending')
                ]

                if (matching_df.empty) :
                    # If no matching receiver, than set the arrow to 'dot',
                    # meaning that only a triangle will be rendered
                    trace_df.loc[index, 'arrow'] = 'dot'
                else:
                    # If there is one or more matching rows, then consider 
                    # the first one, since it is an out -> in arrow, and  
                    # since it is the closet in time
                    # FIXME: What other possible choices to consider?
                    matching_index = matching_df.index[0]
                    matching_row = matching_df.loc[matching_index]
                    # Mark it, so not to consider it anymore
                    # trace_df.at[matching_index, 'arrow'] = 'marked'
                    indexes_to_mark.append(matching_index)
                    trace_df.at[index, 'x2'] = matching_row['x1']
                    trace_df.at[index, 'y2'] = matching_row['y1']
                    if (len(matching_df.index) == 1) :
                        trace_df.at[index, 'arrow'] = 'arrow'
                    else :
                        trace_df.at[index, 'arrow'] = 'dashedarrow'
            else: # 'in' in row['inout']
                # Compute the possible timestamps interval at the receiver side
                physical_time_start = physical_time - network_latency - clock_sync_error
                physical_time_end = physical_time + clock_sync_error 
                
                # Match with 'out' tracepoints
                matching_df = trace_df[\
                    (trace_df['physical_time'] >= physical_time_start) & \
                    (trace_df['physical_time'] <= physical_time_end) & \
                    (trace_df['inout'] == 'out') & \
                    (trace_df['self_id'] == partner_id) & \
                    (trace_df['partner_id'] == self_id) & \
                    (trace_df['arrow'] == 'pending')
                ]

                if (matching_df.empty) :
                    # If no matching receiver, than set the arrow to 'dot',
                    # meaning that only a triangle will be rendered
                    trace_df.loc[index, 'arrow'] = 'dot'
                else : 
                    # If there is one or more matching rows, then consider 
                    # the first one, since it is an out -> in arrow, and  
                    # since it is the closet in time
                    # FIXME: What other possible choices to consider?
                    matching_index = matching_df.index[-1]
                    matching_row = matching_df.loc[matching_index]
                    # Mark it, so not to consider it anymore
                    # trace_df.at[matching_index, 'arrow'] = 'marked'
                    indexes_to_mark.append(matching_index)
                    trace_df.at[index, 'x2'] = trace_df.at[index, 'x1'] 
                    trace_df.at[index, 'y2'] = trace_df.at[index, 'y1'] 
                    trace_df.at[index, 'x1'] = matching_row['x1']
                    trace_df.at[index, 'y1'] = matching_row['y1']
                    if (len(matching_df.index) == 1) :
                        trace_df.at[index, 'arrow'] = 'arrow'
                    else :
                        trace_df.at[index, 'arrow'] = 'dashedarrow'

    ############################################################################
    #### Write to svg file
    ############################################################################
    svg_width = padding + (len(actors) - 1) * spacing + padding + 200
    svg_height = padding + trace_df.iloc[-1]['y1']

    with open('trace_svg.html', 'w', encoding='utf-8') as f:
        # Print header
        f.write('<!DOCTYPE html>\n')
        f.write('<html>\n')
        f.write('<body>\n\n')
        
        f.write('<svg width="'+str(svg_width)+'" height="'+str(svg_height)+'">\n')
        
        # Print the circles and the names
        for key in x_coor:
            if (key == -1):
                f.write(fhlp.svg_string_comment('RTI Actor and line'))
                title = 'RTI'
                center = 15
            else:
                f.write(fhlp.svg_string_comment('Federate '+str(key)+' Actor and line'))
                title = str(key)
                center = 5
            f.write(fhlp.svg_string_draw_line(x_coor[key], math.ceil(padding/2), x_coor[key], svg_height, False))
            f.write('\t<circle cx="'+str(x_coor[key])+'" cy="'+str(math.ceil(padding/2))+'" r="20" stroke="black" stroke-width="2" fill="white"/>\n')
            f.write('\t<text x="'+str(x_coor[key]-center)+'" y="'+str(math.ceil(padding/2)+5)+'" fill="black">'+title+'</text>\n')

        # Now, we need to iterate over the traces to draw the lines
        f.write(fhlp.svg_string_comment('Draw interactions'))
        for index, row in trace_df.iterrows():
            # FIXME: Whose physical and logical time? 
            label = row['event'] + ' @PT=' + str(row['physical_time']) + ' @LT=' + str(row['logical_time'])
            if (row['arrow'] == 'arrow'): 
                f.write(fhlp.svg_string_draw_arrow(row['x1'], row['y1'], row['x2'], row['y2'], label, False))
            elif (row['arrow'] == 'dashedarrow'): 
                f.write(fhlp.svg_string_draw_arrow(row['x1'], row['y1'], row['x2'], row['y2'], label, True))
            elif (row['arrow'] == 'dot'):
                if (row['inout'] == 'in'):
                    label = "(in) from " + str(row['partner_id']) + ' ' + label
                else :
                    label = "(out) to " + str(row['partner_id']) + ' ' + label
                f.write(fhlp.svg_string_draw_dot(row['x1'], row['y1'], label))

        f.write('\n</svg>\n\n')

        # Print footer
        f.write('</body>\n')
        f.write('</html>\n')

    # Write to a csv file, just to double check
    trace_df.to_csv('all.csv', index=True)