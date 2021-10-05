import multiprocessing
from multiprocessing import connection
import threading
import sys
import os
import signal
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

try:
    import pygame
except:
     print("Import Error: Failed to import 'pygame'. Try 'pip3 install pygame'")
     sys.exit(1)


def start_gui():
    user_input_pout, user_input_pin = connection.Pipe(duplex=False)
    update_graphics_pout, update_graphics_pin = connection.Pipe(duplex=False)
    multiprocessing.set_start_method("spawn")
    p = multiprocessing.Process(target=gui, args=(user_input_pin, update_graphics_pout))
    p.start()
    return user_input_pout, update_graphics_pin, p


def gui(user_input_pin, update_graphics_pout):
    pygame.init()
    pygame.font.init()
    g = Gui(user_input=user_input_pin, update_graphics=update_graphics_pout)
    g.start()


class Gui:
    def __init__(self, user_input, update_graphics):
        self.user_input = user_input
        self.update_graphics = update_graphics
        self.font = pygame.font.SysFont("monospace", 14)
        self.size = 320, 240
        self.terminate = False
        self.black = 0, 0, 0
        self.screen = pygame.display.set_mode(self.size)
        self.listener = threading.Thread(target=self.listen_for_update_graphic, args=(update_graphics, ))

    def start(self):
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
                    self.user_input.send(event.unicode)
        except KeyboardInterrupt:
            pygame.quit()
            sys.exit(0)

    def listen_for_update_graphic(self, update_graphics_pout):
        while 1:
            try:
                msg = update_graphics_pout.recv()
            except EOFError:
                pygame.quit()
                return
            self.screen.fill(self.black)
            text_surface = self.font.render(msg, False, (255, 255, 255))
            self.screen.blit(text_surface, (10, 10))
            pygame.display.flip()

