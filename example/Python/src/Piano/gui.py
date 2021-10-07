import multiprocessing
from multiprocessing import Process, connection
from threading import Thread
import sys
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

try:
    import pygame
except:
    print("Import Error: Failed to import 'pygame'. Try 'pip3 install pygame'")
    sys.exit(1)


# sound font and the picture of the keyboard
KEYS_PNG = os.path.join(os.path.dirname(__file__), "keys.png")

# number of octaves to show
OCTAVES = 2

# names of white keys
WHITE_KEYS = ["C", "D", "E", "F", "G", "A", "B"]

# names of black keys
BLACK_KEYS = ["C#", "D#", "F#", "G#", "A#"]

def start_gui(piano_keys):
    '''
    Spawns a process to run the pygame piano.

    Parameters
    ----------
    piano_keys: dict[str] -> tuple(str, int)
        a mapping of keyboard characters to piano notes.

    Returns
    ----------
    multiprocessing.connection.PipeConnection
        a pipe object for the LF program to receive key presses from the pygame process

    multiprocessing.connection.PipeConnection
        a pipe object for the LF program to send graphics update to the pygame process
    '''
    multiprocessing.set_start_method("spawn")
    user_input_pout, user_input_pin = connection.Pipe(duplex=False)
    update_graphics_pout, update_graphics_pin = connection.Pipe(duplex=False)
    p = Process(target=gui, args=(user_input_pin, update_graphics_pout, piano_keys))
    p.start()
    return user_input_pout, update_graphics_pin


def gui(user_input_pin, update_graphics_pout, piano_keys):
    '''
    Starts the pygame piano GUI.

    Parameters
    ----------
    user_input_pin : multiprocessing.connection.PipeConnection
        a pipe object for the pygame piano to send key press signals to the LF program.
    update_graphics_pout: multiprocessing.connection.PipeConnection
        a pipe object for the pygame piano to receive acutations from the LF program.
    piano_keys: dict[str] -> tuple(str, int)
        a mapping of keyboard characters to piano notes.

    Returns
    ----------
    None
    '''
    gui = PianoGui(user_input_pin, update_graphics_pout, piano_keys)
    gui.start()


class PianoGui:
    def __init__(self, user_input_pin, update_graphics_pout, piano_keys):
        '''
        Initialize the screen of the pygame piano.

        Parameters
        ----------
        user_input_pin : multiprocessing.connection.PipeConnection
            a pipe object for the pygame piano to send key press signals to the LF program.
        update_graphics_pout: multiprocessing.connection.PipeConnection
            a pipe object for the pygame piano to receive acutations from the LF program.
        piano_keys: dict[str] -> tuple(str, int)
            a mapping of keyboard characters to piano notes.

        Returns
        ----------
        None
        '''
        self.user_input_pin = user_input_pin
        self.update_graphics_pout = update_graphics_pout
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
        pygame.display.set_caption("piano demo")

        # pressed is a surface that is used to show where a key has been pressed
        self.pressed = pygame.Surface((self.white_key_width, self.height))
        self.pressed.fill((0, 230, 0))


    def get_note_coordinate(self, note_name, octave):
        '''
        Calculates the x-coordinate of the note on the screen of the pygame piano

        Parameters
        ----------
        note_name: str
            the name of the note (Ex. "C", "D#", etc...).
        octave: int
            the number of octave(s) above the lowest "C" note on the pygame piano.

        Returns
        ----------
        int
            the x-coordinate of the Note object on the screen.
        '''
        octave_offset = octave * self.width
        if note_name in WHITE_KEYS:
            coordinate = WHITE_KEYS.index(note_name) * self.white_key_width + octave_offset
        else:
            i = BLACK_KEYS.index(note_name)
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


    def listen_for_graphics_update(self):
        '''
        Receives graphics update from the LF program.
            Redraw the screen upon receiving graphics update.

        Parameters
        ----------
        None

        Returns
        ----------
        None
        '''
        while 1:
            try:
                playing_notes = self.update_graphics_pout.recv()
            except EOFError:
                pygame.quit()
                return
            self.blit_keyboard()

            # draw the pressed white keys
            for note_name, octave in playing_notes:
                if note_name in WHITE_KEYS:
                    self.screen.blit(self.pressed, (self.get_note_coordinate(note_name, octave), 0), None, pygame.BLEND_SUB)

            # draw the pressed black keys
            for note_name, octave in playing_notes:
                if note_name in BLACK_KEYS:
                    self.screen.blit(self.pressed, (self.get_note_coordinate(note_name, octave), 1), (0, 0, 19, 68), pygame.BLEND_ADD)
            pygame.display.flip()


    def blit_keyboard(self):
        '''
        Draws the piano keyboard and the key presses for each note.

        Parameters
        ----------
        None

        Returns
        ----------
        None
        '''

        # Displaying the keyboard
        for i in range(OCTAVES):
            self.screen.blit(self.key_graphic, (i * self.width, 0))

            # Displaying the text for each white key
            for j, white_key in enumerate(WHITE_KEYS):
                key_surface = self.font.render(self.note_to_key[(white_key, i)], False, (0, 0, 0))
                self.screen.blit(key_surface,
                                 (self.get_note_coordinate(white_key, i) + self.white_key_width / 2, self.height * 0.8))

            # Displaying the text for each black key
            for j, black_key in enumerate(BLACK_KEYS):
                key_surface = self.font.render(self.note_to_key[(black_key, i)], False, (255, 255, 255))
                self.screen.blit(key_surface,
                                 (self.get_note_coordinate(black_key, i) + self.white_key_width / 3, self.height * 0.2))


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
        self.blit_keyboard()
        pygame.display.flip()

        listener = Thread(target=self.listen_for_graphics_update)
        listener.daemon = True
        listener.start()

        try:
            while 1:
                event = pygame.event.wait()
                if event.type == pygame.QUIT:
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    self.user_input_pin.send((True,           # Key Down == True
                                              event.unicode))
                elif event.type == pygame.KEYUP:
                    self.user_input_pin.send((False,
                                              event.unicode))
        except KeyboardInterrupt:
            pygame.quit()
            sys.exit(0)
