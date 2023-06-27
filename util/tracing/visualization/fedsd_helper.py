import math

# Disctionary for pruning event names. Usefule for tracepoint matching and
# communication rendering
prune_event_name = {
    "Sending ACK": "ACK",
    "Sending TIMESTAMP": "TIMESTAMP",
    "Sending NET": "NET",
    "Sending LTC": "LTC",
    "Sending STOP_REQ": "STOP_REQ",
    "Sending STOP_REQ_REP": "STOP_REQ_REP",
    "Sending STOP_GRN": "STOP_GRN",
    "Sending FED_ID": "FED_ID",
    "Sending PTAG": "PTAG",
    "Sending TAG": "TAG",
    "Sending REJECT": "REJECT",
    "Sending RESIGN": "RESIGN",
    "Sending PORT_ABS": "ABS",
    "Sending CLOSE_RQ": "CLOSE_RQ",
    "Sending TAGGED_MSG": "T_MSG",
    "Sending P2P_TAGGED_MSG": "P2P_T_MSG",
    "Sending MSG": "MSG",
    "Sending P2P_MSG": "P2P_MSG",
    "Sending ADR_AD": "ADR_AD",
    "Sending ADR_QR": "ADR_QR",
    "Receiving ACK": "ACK",
    "Receiving TIMESTAMP": "TIMESTAMP",
    "Receiving NET": "NET",
    "Receiving LTC": "LTC",
    "Receiving STOP_REQ": "STOP_REQ",
    "Receiving STOP_REQ_REP": "STOP_REQ_REP",
    "Receiving STOP_GRN": "STOP_GRN",
    "Receiving FED_ID": "FED_ID",
    "Receiving PTAG": "PTAG",
    "Receiving TAG": "TAG",
    "Receiving REJECT": "REJECT",
    "Receiving RESIGN": "RESIGN",
    "Receiving PORT_ABS": "ABS",
    "Receiving CLOSE_RQ": "CLOSE_RQ",
    "Receiving TAGGED_MSG": "T_MSG",
    "Receiving P2P_TAGGED_MSG": "P2P_T_MSG",
    "Receiving MSG": "MSG",
    "Receiving P2P_MSG": "P2P_MSG",
    "Receiving ADR_AD": "ADR_AD",
    "Receiving ADR_QR": "ADR_QR",
    "Receiving UNIDENTIFIED": "UNIDENTIFIED",
    "Scheduler advancing time ends": "AdvLT"
}

prune_event_name.setdefault(" ", "UNIDENTIFIED")

################################################################################
### Routines to write to csv file
################################################################################

def svg_string_draw_line(x1, y1, x2, y2, type=''):
    '''
    Constructs the svg html string to draw a line from (x1, y1) to (x2, y2).

    Args:
     * x1: Int X coordinate of the source point
     * y1: Int Y coordinate of the source point
     * x2: Int X coordinate of the sink point
     * y2: Int Y coordinate of the sink point
     * type: The type of the message (for styling)
    Returns:
     * String: the svg string of the line©
    '''
    str_line = '\t<line x1="'+str(x1)+'" y1="'+str(y1)+'" x2="'+str(x2)+'" y2="'+str(y2)+'"'
    if (type):
            str_line = str_line + ' class="' + type + '"'
 
    str_line = str_line +  '/>\n'
    return str_line


def svg_string_draw_arrow_head(x1, y1, x2, y2, type='') :
    '''
    Constructs the svg html string to draw the arrow end

    Args:
     * x1: Int X coordinate of the source point
     * y1: Int Y coordinate of the source point
     * x2: Int X coordinate of the sink point
     * y2: Int Y coordinate of the sink point
     * type: The type (for styling)
    Returns:
     * String: the svg string of the triangle
    '''

    if (y2 != y1):
        rotation = - math.ceil(math.atan((x2-x1)/(y2-y1)) * 180 / 3.14) - 90
    else:
        if (x1 > x2):
            rotation = 0
        else:
            rotation = - 180
        
    style = ''
    if (type):
        style = ' class="'+type+'"'
    
    str_line = ''
    if (x1 > x2) :
        str_line = '\t<path d="M'+str(x2)+' '+str(y2)+' L'+str(x2+10)+' '+str(y2+5)+' L'+str(x2+10)+' '+str(y2-5)+' Z"' \
             + ' transform="rotate('+str(rotation)+')" transform-origin="'+str(x2)+' '+str(y2)+'"' \
             + style \
             + '/>\n'
    else :
        str_line = '\t<path d="M'+str(x2)+' '+str(y2)+' L'+str(x2-10)+' '+str(y2+5)+' L'+str(x2-10)+' '+str(y2-5)+' Z"' \
             + ' transform="rotate('+str( 180 + rotation)+')" transform-origin="'+str(x2)+' '+str(y2)+'"' \
             + style \
             + '/>\n'

    return str_line


def svg_string_draw_label(x1, y1, x2, y2, label) :
    '''
    Computes the rotation angle of the text and then constructs the svg string. 

    Args:
     * x1: Int X coordinate of the source point
     * y1: Int Y coordinate of the source point
     * x2: Int X coordinate of the sink point
     * y2: Int Y coordinate of the sink point
     * label: The label to draw
    Returns:
     * String: the svg string of the text
    '''
    # FIXME: Need further improvement, based of the position of the arrows
    # FIXME: Rotation value is not that accurate. 
    if (x2 < x1) :
        # Left-going arrow.
        if (y2 != y1):
            rotation = - math.ceil(math.atan((x2-x1)/(y2-y1)) * 180 / 3.14) - 90
        else:
            rotation = 0

        str_line = '\t<text text-anchor="end" transform="translate('+str(x1-10)+', '+str(y1-5)+') rotate('+str(rotation)+')">'+label+'</text>\n'
    else :
        # Right-going arrow.
        if (y2 != y1):
            rotation = - math.ceil(math.atan((x1-x2)/(y1-y2)) * 180 / 3.14) + 90
        else:
            rotation = 0
        str_line = '\t<text transform="translate('+str(x1+10)+', '+str(y1-5)+') rotate('+str(rotation)+')" text-anchor="start">'+label+'</text>\n'
    #print('rot = '+str(rotation)+' x1='+str(x1)+' y1='+str(y1)+' x2='+str(x2)+' y2='+str(y2))
    return str_line


def svg_string_draw_arrow(x1, y1, x2, y2, label, type=''):
    '''
    Constructs the svg html string to draw the arrow from (x1, y1) to (x2, y2). 
    The arrow end is constructed, together with the label

    Args:
     * x1: Int X coordinate of the source point
     * y1: Int Y coordinate of the source point
     * x2: Int X coordinate of the sink point
     * y2: Int Y coordinate of the sink point
     * label: String Label to draw on top of the arrow
     * type: The type of the message
    Returns:
     * String: the svg string of the arrow
    '''
    str_line1 = svg_string_draw_line(x1, y1, x2, y2, type)
    str_line2 = svg_string_draw_arrow_head(x1, y1, x2, y2, type)
    str_line3 = svg_string_draw_label(x1, y1, x2, y2, label)
    return str_line1 + str_line2 + str_line3

def svg_string_draw_side_label(x, y, label, anchor="start") :
    '''
    Put a label to the right of the x, y point,
    unless x is small, in which case put it to the left.

    Args:
     * x: Int X coordinate of the source point
     * y: Int Y coordinate of the source point
     * label: Label to put by the point.
     * anchor: One of "start", "middle", or "end" to specify the text-anchor.
    Returns:
     * String: the svg string of the text
    '''
    offset = 5
    if (anchor == 'end'):
        offset = -5
    elif (anchor == 'middle'):
        offset = 0
    str_line = '\t<text text-anchor="'+anchor+'"' \
    +' class="time"' \
    +' transform="translate('+str(x+offset)+', '+str(y+5)+')">'+label+'</text>\n'

    return str_line

def svg_string_comment(comment):
    '''
    Constructs the svg html string to write a comment into an svg file.

    Args:
     * comment: String Comment to add
    Returns:
     * String: the svg string of the comment
    '''
    str_line = '\n\t<!-- ' + comment + ' -->\n'
    return str_line


def svg_string_draw_dot(x, y, label) :
    '''
    Constructs the svg html string to draw at a dot.

    Args:
     * x: Int X coordinate of the dot
     * y: Int Y coordinate of the dot
     * label: String to draw 
    Returns:
     * String: the svg string of the triangle
    '''
    str_line = ''
    str_line = '\t<circle cx="'+str(x)+'" cy="'+str(y)+'" r="3" stroke="black" stroke-width="1" fill="black"/>\n'
    str_line = str_line + '\t<text x="'+str(x+5)+'", y="'+str(y+5)+'" fill="blue">'+label+'</text>\n'
    return str_line

def svg_string_draw_dot_with_time(x, y, time, label) :
    '''
    Constructs the svg html string to draw at a dot with a prefixed physical time.

    Args:
     * x: Int X coordinate of the dot
     * y: Int Y coordinate of the dot
     * time: The time
     * label: String to draw 
    Returns:
     * String: the svg string of the triangle
    '''
    str_line = ''
    str_line = '\t<circle cx="'+str(x)+'" cy="'+str(y)+'" r="3" stroke="black" stroke-width="1" fill="black"/>\n'
    str_line = str_line + '\t<text x="'+str(x+5)+'", y="'+str(y+5)+'"> <tspan class="time">'+time+':</tspan> <tspan fill="blue">'+label+'</tspan></text>\n'
    return str_line

def svg_string_draw_adv(x, y, label) :
    '''
    Constructs the svg html string to draw at a dash, meaning that logical time is advancing there.

    Args:
     * x: Int X coordinate of the dash
     * y: Int Y coordinate of the dash
     * label: String to draw 
    Returns:
     * String: the svg string of the triangle
    '''
    str_line1 = svg_string_draw_line(x-5, y, x+5, y, "ADV")
    str_line2 = svg_string_draw_side_label(x, y, label)
    return str_line1 + str_line2