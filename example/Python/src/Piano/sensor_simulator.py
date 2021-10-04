from multiprocessing import Process, connection
from threading import Thread
import sys
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

try:
    from mingus.containers.note import Note
except:
    print("Import Error: Failed to import 'mingus'. Try 'pip3 install mingus'")
    sys.exit(1)

try:
    from mingus.midi import fluidsynth
except:
    if sys.platform == "darwin":
        print("Import Error: fluidsynth is missing. Try 'brew install fluidsynth'")
    elif sys.platform == "linux" or sys.platform == "linux2":
        print("Import Error: fluidsynth is missing. Try 'sudo apt-get install -y fluidsynth'")
    else:
        print("Import Error: fluidsynth is missing. ")

try:
    import pygame
except:
    print("Import Error: Failed to import 'pygame'. Try 'pip3 install pygame'")
    sys.exit(1)


# sound font and the picture of the keyboard
SF2 = os.path.join(os.path.dirname(__file__), "soundfont.sf2")
KEYS_PNG = os.path.join(os.path.dirname(__file__), "keys.png")

# number of octaves to show
OCTAVES = 2

# lowest octave to show 
LOWEST = 4

# names of white keys
WHITE_KEYS = ["C", "D", "E", "F", "G", "A", "B"]

# names of black keys
BLACK_KEYS = ["C#", "D#", "F#", "G#", "A#"]

def start_sensor_simulator(action, piano_keys):
    '''
    spawns a process to run the pygame piano and 
        a thread to receive key press signals from 
        the pygame piano.

    Parameters
    ----------
    action : physical action
        a physical action passed in from the Piano reactor.

    Returns
    ----------
    multiprocessing.connection.PipeConnection
        a pipe object for the LF program to send actuations to the pygame piano.
    '''
    sensor_pout, sensor_pin = connection.Pipe(duplex=False)
    actuator_pout, actuator_pin = connection.Pipe(duplex=False)
    t1 = Thread(target=sensor_receiver, args=(action, sensor_pout))
    p = Process(target=start_gui, args=(sensor_pin, actuator_pout, piano_keys))
    t1.start()
    p.start()
    return actuator_pin


def sensor_receiver(action, pout):
    '''
    receives signal from the pygame piano
        and schedule a physical action for the lf program. 

    Parameters
    ----------
    action : physical action
        a physical action passed in from the PianoGui reactor.
    pout: multiprocessing.connection.PipeConnection
        a pipe object for the LF program to receive key press signals from the pygame piano.

    Returns
    ----------
    None
    '''
    while 1:
        key_packet = pout.recv()
        if key_packet == None:
            action.schedule(0, None)
            return
        unicode, = key_packet.content
        print("sensor_receiver: scheduling key action " + unicode)
        action.schedule(0, key_packet)


def start_gui(sensor_pin, actuator_pout, piano_keys):
    '''
    starts the pygame piano

    Parameters
    ----------
    sensor_pin : multiprocessing.connection.PipeConnection
        a pipe object for the pygame piano to send key press signals to the LF program.
    actuator_pout: multiprocessing.connection.PipeConnection
        a pipe object for the pygame piano to receive acutations from the LF program.

    Returns
    ----------
    None
    '''
    gui = PianoGui(sensor_pin, actuator_pout, piano_keys)
    gui.start()


class KeyPacket:
    def __init__(self, key_down, content):
        '''
        Formats information about sensor data or an actuation.

        Parameters
        ----------
        key_down : Boolean
            True if packet describes a key press or a start actuation.
            False if packet describes a key up or a end actuation.
        content: tuple
            a tuple of data describing the key press/up or actuation.

        Returns
        ----------
        None
        '''
        self.key_down = key_down
        self.content = content


class PianoGui:
    def __init__(self, sensor_pin, actuator_pout, piano_keys):
        '''
        initialize the screen of the pygame piano.

        Parameters
        ----------
        sensor_pin : multiprocessing.connection.PipeConnection
            a pipe object for the pygame piano to send key press signals to the LF program.
        actuator_pout: multiprocessing.connection.PipeConnection
            a pipe object for the pygame piano to receive acutations from the LF program.

        Returns
        ----------
        None
        '''
        driver = None
        if sys.platform == "linux" or sys.platform == "linux2":
            driver = "alsa"
        if not fluidsynth.init(SF2, driver):
            print("Couldn't load soundfont", SF2)
            sys.exit(1)
        self.sensor_pin = sensor_pin
        self.actuator_pout = actuator_pout
        self.note_to_key = {v: k for k, v in piano_keys.items()}

        pygame.init()
        pygame.font.init()
        self.font = pygame.font.SysFont("monospace", 14)
        self.screen = pygame.display.set_mode((640, 480))
        self.image = pygame.image.load(KEYS_PNG).convert()
        (self.key_graphic, self.kgrect) = (self.image, self.image.get_rect())
        (self.width, self.height) = (self.kgrect.width, self.kgrect.height)
        self.white_key_width = self.width / 7

        # Reset display to wrap around the keyboard image
        pygame.display.set_mode((OCTAVES * self.width, self.height))
        pygame.display.set_caption("mingus piano")
        self.octave = LOWEST
        self.channel = 8

        # pressed is a surface that is used to show where a key has been pressed
        self.pressed = pygame.Surface((self.white_key_width, self.height))
        self.pressed.fill((0, 230, 0))

        # white keys being played right now
        self.playing_w = []

        # black keys being played right now
        self.playing_b = []


    def get_note_coordinate(self, note):
        '''
        calculates the x-coordinate of the note on the screen of the pygame piano

        Parameters
        ----------
        note: mingus.containers.note.Note
            a Note object

        Returns
        ----------
        int
            the x-coordinate of the Note object on the screen.
        '''
        octave_offset = (note.octave - LOWEST) * self.width
        if note.name in WHITE_KEYS:
            # Getting the x coordinate of a white key can be done automatically
            coordinate = WHITE_KEYS.index(note.name) * self.white_key_width + octave_offset

            # Add a list containing the x coordinate, the tick at the current time
            # and of course the note itself to playing_w
        else:
            # For black keys I hard coded the x coordinates. It's ugly.
            i = BLACK_KEYS.index(note.name)
            if i == 0:
                coordinate = 18
            elif i == 1:
                coordinate = 58
            elif i == 2:
                coordinate = 115
            elif i == 3:
                coordinate = 151
            else:
                coordinate = 187
            coordinate += octave_offset
        return coordinate


    def play_note(self, note):
        '''
        starts playing note

        Parameters
        ----------
        note: mingus.containers.note.Note
            a Note object

        Returns
        ----------
        None
        '''
        coordinate = self.get_note_coordinate(note)
        if note.name in WHITE_KEYS:
            self.playing_w.append((coordinate, note))
        else:
            self.playing_b.append((coordinate, note))
        fluidsynth.play_Note(note, self.channel, 100)


    def stop_note(self, note):
        '''
        stops playing note

        Parameters
        ----------
        note: mingus.containers.note.Note
            a Note object

        Returns
        ----------
        None
        '''
        coordinate = self.get_note_coordinate(note)
        if note.name in WHITE_KEYS:
            self.playing_w.remove((coordinate, note))
        else:
            self.playing_b.remove((coordinate, note))
        fluidsynth.stop_Note(note, self.channel)


    def actuator_receiver(self):
        '''
        stops playing note

        Parameters
        ----------
        note: mingus.containers.note.Note
            a Note object

        Returns
        ----------
        None
        '''
        while 1:
            key_packet = self.actuator_pout.recv()
            if key_packet == None:
                return
            note_name, octave = key_packet.content
            print("actuator_receiver: received " + note_name)
            if key_packet.key_down:
                self.play_note(Note(note_name, self.octave + octave))
            else:
                self.stop_note(Note(note_name, self.octave + octave))


    def start(self):
        '''
        starts the pygame piano loop

        Parameters
        ----------
        None

        Returns
        ----------
        None
        '''
        print('pygame: starting actuator_receiver thread')
        t = Thread(target=self.actuator_receiver)
        t.start()

        while 1:
            # Displaying the keyboard
            for i in range(OCTAVES):
                self.screen.blit(self.key_graphic, (i * self.width, 0))

                # Displaying the text for each white key
                for j, white_key in enumerate(WHITE_KEYS):
                    key_surface = self.font.render(self.note_to_key[(white_key, i)], False, (0, 0, 0))
                    note_object = Note(white_key, self.octave + i)
                    self.screen.blit(key_surface, (self.get_note_coordinate(note_object) + self.white_key_width / 2, self.height * 0.8))

                # Displaying the text for each black key
                for j, black_key in enumerate(BLACK_KEYS):
                    key_surface = self.font.render(self.note_to_key[(black_key, i)], False, (255, 255, 255))
                    note_object = Note(black_key, self.octave + i)
                    self.screen.blit(key_surface, (self.get_note_coordinate(note_object) + self.white_key_width / 3, self.height * 0.2))

            # Highlight each white key being played
            for note in self.playing_w:
                self.screen.blit(self.pressed, (note[0], 0), None, pygame.BLEND_SUB)

            # Highlight each black key being played
            for note in self.playing_b:
                self.screen.blit(self.pressed, (note[0], 1), (0, 0, 19, 68), pygame.BLEND_ADD)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.sensor_pin.send(None)
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    print('pygame: key down: ' + event.unicode)
                    self.sensor_pin.send(KeyPacket(key_down=True, content=(event.unicode, )))
                elif event.type == pygame.KEYUP:
                    print('pygame: key up: ' + event.unicode)
                    self.sensor_pin.send(KeyPacket(key_down=False, content=(event.unicode, )))

            pygame.display.update()
