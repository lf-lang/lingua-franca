import curses

class Logger:
    def __init__(self, max_lines=3):
        self.max_lines = max_lines
        self.log = []

    def append_log(self, msg):
        self.log.append(msg)
        if len(self.log) > self.max_lines:
            self.log.pop(0)
    
    def get_log(self):
        return self.log

    def log_size(self):
        return len(self.log)


class Window:
    def __init__(self):
        self.window = curses.initscr()
        curses.cbreak()
        curses.noecho()
        self.window.keypad(True)

    def change_line(self, y, new_msg):
        self.window.addstr(y, 0, new_msg)
        self.window.clrtoeol()
    
    def refresh(self):
        self.window.refresh()

    def getch(self):
        return self.window.getch()

