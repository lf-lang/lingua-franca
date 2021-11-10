import multiprocessing
from multiprocessing import connection
import threading
import sys
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

try:
    import pygame
except:
     print("Import Error: Failed to import 'pygame'. Try 'pip3 install pygame'")
     sys.exit(1)


def start_gui():
    '''
    Spawns a new process that runs the pygame gui.

    Parameters
    ----------
    None

    Returns
    ----------
    multiprocessing.connection.PipeConnection
        a Pipe object for the pygame process to send user input to the LF program
    multiprocessing.connection.PipeConnection
        a Pipe object for the LF program to send graphics update to the pygame process
    '''
    user_input_pout, user_input_pin = connection.Pipe(duplex=False)
    update_graphics_pout, update_graphics_pin = connection.Pipe(duplex=False)
    multiprocessing.set_start_method("spawn")
    p = multiprocessing.Process(target=gui, args=(user_input_pin, update_graphics_pout))
    p.start()
    return user_input_pout, update_graphics_pin


def gui(user_input_pin, update_graphics_pout):
    '''
    Initializes a Gui class and starts the gui.

    Parameters
    ----------
    user_input_pin : multiprocessing.connection.PipeConnection
        a Pipe object for the pygame process to send user input to the LF program
    update_graphics_pout : multiprocessing.connection.PipeConnection
        a Pipe object for the LF program to send graphics update to the pygame process

    Returns
    ----------
    None
    '''
    pygame.init()
    pygame.font.init()
    g = Gui(user_input_pin=user_input_pin, update_graphics_pout=update_graphics_pout)
    g.start()


class Gui:
    def __init__(self, user_input_pin, update_graphics_pout):
        '''
        Initializes the pygame window and spawns a thread to listen for graphics update from the LF program.

        Parameters
        ----------
        user_input_pin : multiprocessing.connection.PipeConnection
            a Pipe object for the pygame process to send user input to the LF program
        update_graphics_pout : multiprocessing.connection.PipeConnection
            a Pipe object for the LF program to send graphics update to the pygame process

        Returns
        ----------
        None
        '''
        self.user_input_pin = user_input_pin
        self.font = pygame.font.SysFont("arial", 18)
        self.size = self.width, self.height = 500, 500
        self.terminate = False
        self.black = 0, 0, 0
        self.screen = pygame.display.set_mode(self.size)
        self.listener = threading.Thread(target=self.listen_for_update_graphic, args=(update_graphics_pout, ))

    def start(self):
        '''
        Draw the pygame window and listen for user input.
        Sends user input to the LF program using self.user_input upon detection.

        Parameters
        ----------
        None

        Returns
        ----------
        None
        '''
        self.screen.fill(self.black)
        pygame.display.flip()
        self.listener.daemon = True
        self.listener.start()
        try:
            while 1:
                event = pygame.event.wait()
                if event.type == pygame.QUIT:
                    sys.exit(0)
                elif event.type == pygame.KEYDOWN:
                    self.user_input_pin.send(event.unicode)
        except KeyboardInterrupt:
            pygame.quit()
            sys.exit(0)

    def listen_for_update_graphic(self, update_graphics_pout):
        '''
        Listens for graphics update from the LF program.
        Redraw the pygame window upon receiving graphics update.

        Parameters
        ----------
        update_graphics_pout : multiprocessing.connection.PipeConnection
            a Pipe object for the LF program to send graphics update to the pygame process

        Returns
        ----------
        None
        '''
        while 1:
            try:
                bg_color, text_color, *msg = update_graphics_pout.recv()
            except EOFError:
                pygame.quit()
                return

            self.screen.fill(bg_color)
            for i, text in enumerate(msg):
                text_surface = self.font.render(text, False, text_color)
                text_height = text_surface.get_height()
                text_rect = text_surface.get_rect(center=(self.width // 2,
                                                          self.height // 2 + text_height * (0.5 + i - 0.5 * len(msg))))
                self.screen.blit(text_surface, text_rect)
            pygame.display.flip()

