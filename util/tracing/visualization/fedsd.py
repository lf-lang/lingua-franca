#!/usr/bin/env python3
import argparse         # For arguments parsing
import pandas as pd     # For csv manipulation
from os.path import exists
import math

# Define the arguments to pass in the command line
parser = argparse.ArgumentParser(description='Set of the csv trace files to render.')
parser.add_argument('-r','--rti', type=str, default="rti.csv",
                    help='RTI csv trace file.')
parser.add_argument('-f','--federates', nargs='+', action='append',
                    help='List of the federates csv trace files.')


def prune_event_name(event_name) :
    '''
    Prunes the event name, so that to get nice to render string on top of 
    the arrows.

    Args:
     * event_name: String with the event name
    Returns:
     * pruned event name  
    '''
    tmp_str = event_name
    if ('RTI accepts joining federate' in event_name) :
        tmp_str = "JOIN"
    elif ('RTI receives ' in event_name) :
        tmp_str = event_name.split('RTI receives ')[1]
        tmp_str = tmp_str.split(' from federate')[0]
    elif ('RTI sends ' in event_name) :
        tmp_str = event_name.split('RTI sends ')[1]
        tmp_str = tmp_str.split(' to federate')[0]
    elif ('Federate receives ' in event_name) :
        tmp_str = event_name.split('Federate receives ')[1]
        tmp_str = tmp_str.split(' from RTI')[0]
    elif ('Federate sends ' in event_name) :
        tmp_str = event_name.split('Federate sends ')[1]
        tmp_str = tmp_str.split(' to RTI')[0]

    return tmp_str


def svg_string_draw_line(x1, y1, x2, y2):
    '''
    Constructs the svg html string to draw a line from (x1, y1) to (x2, y2)
    '''
    str_line = '\t<line x1="'+str(x1)+'" y1="'+str(y1)+'" x2="'+str(x2)+'" y2="'+str(y2)+'" stroke="black" stroke-width="2" />\n'
    return str_line


def svg_string_draw_arrow(x1, y1, x2, y2):
    '''
    Constructs the svg html string to draw a line from (x1, y1) to (x2, y2)
    '''
    str_line1 = svg_string_draw_line(x1, y1, x2, y2)
    str_line2 = ''
    if (x1 > x2) :
        str_line2 = '\t<path d="M'+str(x2)+' '+str(y2)+' L'+str(x2+10)+' '+str(y2+5)+' L'+str(x2+10)+' '+str(y2-5)+' Z" />\n'
    else :
        str_line2 = '\t<path d="M'+str(x2)+' '+str(y2)+' L'+str(x2-10)+' '+str(y2+5)+' L'+str(x2-10)+' '+str(y2-5)+' Z" />\n'
    
    return str_line1 + str_line2

def svg_string_comment(string):
    '''
    Constructs the svg html string to write a comment into an svg file
    '''
    str_line = '\n\t<!-- ' + string + ' -->\n'
    return str_line





if __name__ == '__main__':
    args = parser.parse_args()

    # Check if the files exist
    if (not exists(args.rti)):
        print('Error: No RTI csv tarce file!')
        exit(0)
    
    ############################################################################
    #### RTI trace processing
    ############################################################################

    # Load RTI tracepoints, rename the columns and clean non useful data
    trace_df = pd.read_csv(args.rti)
    trace_df.columns = ['event', 'r', 'fed_id', 'w', 'logical_time', 'm', 'physical_time', 't', 'ed']
    trace_df = trace_df.drop(columns=['r', 'w', 'm', 't', 'ed'])

    # Remove all the lines that do not contain communication information
    # which boils up to having 'RTI' in the 'event' column
    trace_df = trace_df[trace_df['event'].str.contains('RTI') == True]

    # Add an inout column to set the arrow direction
    trace_df['inout'] = trace_df['event'].apply(lambda e: 'in' if 'receives' in e else 'out')

    # Prune event names
    trace_df['event'] = trace_df['event'].apply(lambda e: prune_event_name(e))

    # Set that these are the RTI information, by setting 
    trace_df['rti'] = True
    # print(trace_df)

    # Count the number of actors
    actors_nbr = 1

    ############################################################################
    #### Federates trace processing
    ############################################################################
    # Loop over the given list of federates trace files 
    if (args.federates) :
        for fed_trace in args.federates[0]:
            print(fed_trace)
            if (not exists(fed_trace)):
                print('Warning: Trace file ' + fed_trace + ' does not exist! Will resume though')
                continue

            # Proceed as done with the RTI
            fed_df = pd.read_csv(fed_trace)
            fed_df.columns = ['event', 'r', 'fed_id', 'w', 'logical_time', 'm', 'physical_time', 't', 'ed']
            fed_df = fed_df.drop(columns=['r', 'w', 'm', 't', 'ed'])
            fed_df = fed_df[fed_df['event'].str.contains('RTI') == True]
            fed_df['inout'] = fed_df['event'].apply(lambda e: 'in' if 'receives' in e else 'out')
            fed_df['event'] = fed_df['event'].apply(lambda e: prune_event_name(e))
            fed_df['rti'] = False
            # print(fed_df)
            actors_nbr = actors_nbr + 1

            # Append into trace_df
            trace_df = trace_df.append(fed_df, sort=False, ignore_index=True)

    # Sort all traces by physical time and then reset the index
    trace_df = trace_df.sort_values(by=['physical_time'])
    trace_df = trace_df.reset_index(drop=True)

    # FIXME: For now, we need to remove the rows with negative physical time values...
    # Until the reason behinf such values is investigated. The negative physical
    # time is when federates are still in the process of joining
    trace_df = trace_df[trace_df['physical_time'] >= 0]

    # Add the Y column and initialize it with 0
    trace_df['y'] = 50 # Or set a small shift

    ############################################################################
    #### Process the traces in order to create the 'Y' coordinates
    ############################################################################
    ppt = 0     # Previous physical time
    cpt = 0     # Current physical time
    py = 0      # Previous y
    min = 10    # Will probably be set manually
    scale = 1   # Will probably be set manually
    for index, row in trace_df.iterrows():
        if (index != 2) :
            cpt = int(row['physical_time'])
            # print('cpt = '+str(cpt)+' and ppt = '+ppt)
            # From the email:
            # Y = T_previous + min + log10(1 + (T - T_previous)*scale)
            # But rather think it should be:
            py = math.ceil(py + min + (1 + math.log10(cpt - ppt) * scale))
            trace_df.at[index, 'y'] = py

        ppt = int(row['physical_time'])
        py = trace_df.at[index, 'y']

    ############################################################################
    #### Compute the 'X' coordinates
    ############################################################################
    spacing = 200       # Spacing between actors
    padding = 50
    svg_width = padding + (actors_nbr - 1) * spacing + padding
    svg_height = padding + int(trace_df.iloc[-1]['y'])
    x_rti = 50
    x_fed = []
    for i in range(0, actors_nbr-1) :
        x_fed.append(padding + (spacing * (i+1)))

    # Write all the x coordinates
    trace_df['x'] = trace_df[['rti','fed_id']].apply(lambda x: x_rti if x['rti'] == True else x_fed[int(x['fed_id'])], axis=1)

    #
    # FIXME: Should add processing to match the communications...
    # Currently, everything is duplicated!!!
    #

    ############################################################################
    #### Write to svg file
    ############################################################################
    with open('trace_svg.html', 'w', encoding='utf-8') as f:
        # Print header
        f.write('<!DOCTYPE html>\n')
        f.write('<html>\n')
        f.write('<body>\n\n')
        
        f.write('<svg width="'+str(svg_width)+'" height="'+str(svg_height)+'">\n')
        
        # Print the circles and the names
        # RTI
        f.write(svg_string_comment('RTI Actor and line'))
        f.write(svg_string_draw_line(x_rti, math.ceil(padding/2), x_rti, svg_height))
        f.write('\t<circle cx="'+str(x_rti)+'" cy="'+str(math.ceil(padding/2))+'" r="20" stroke="black" stroke-width="2" fill="black"/>\n')
        f.write('\t<text x="'+str(x_rti-15)+'" y="'+str(math.ceil(padding/2)+5)+'" fill="red">RTI</text>\n')
        # Federates
        for i in range(0, actors_nbr-1):
            f.write(svg_string_comment('Federate '+str(i)+' Actor and line'))
            f.write(svg_string_draw_line(x_fed[i], math.ceil(padding/2), x_fed[i], svg_height))
            f.write('\t<circle cx="'+str(x_fed[i])+'" cy="'+str(math.ceil(padding/2))+'" r="20" stroke="black" stroke-width="2" fill="white"/>\n')
            f.write('\t<text x="'+str(x_fed[i]-5)+'" y="'+str(math.ceil(padding/2)+5)+'" fill="red">'+str(i)+'</text>\n')

        # Now, we need to iterate over the traces to draw the lines
        #
        # FIXME: Here, we draw every single message, w/o checking the connection
        # and the arrival time. This can be done as pre-processing.
        # This means that most arrows are duplicated :-/
        #
        f.write(svg_string_comment('Draw interactions'))
        for index, row in trace_df.iterrows():
            if (row['rti'] == True):
                if ('out' in row['inout']): # RTI -> Federate
                    f.write(svg_string_draw_arrow(x_rti, int(row['y']), x_fed[int(row['fed_id'])], int(row['y'])))
                else: # Federate -> RTI
                    f.write(svg_string_draw_arrow(x_fed[int(row['fed_id'])], int(row['y']), x_rti, int(row['y'])))
            else:
                if ('out' in row['inout']): # Federate -> RTI
                    f.write(svg_string_draw_arrow(x_fed[int(row['fed_id'])], int(row['y']), x_rti, int(row['y'])))
                else: # RTI -> Federate
                    f.write(svg_string_draw_arrow(x_rti, int(row['y']), x_fed[int(row['fed_id'])], int(row['y'])))

        f.write('</svg>\n\n')

        # Print footer
        f.write('</body>\n')
        f.write('</html>\n')


    # Write to a csv file, just to double check
    trace_df.to_csv('all.csv', index=False)