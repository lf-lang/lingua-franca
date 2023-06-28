class Tag:
    def __init__(self, time=0, microstep=0):
        self.time = time
        self.microstep = microstep

    def __str__(self):
        return f"({self.time}, {self.microstep})"

    def __add__(self, other):
        if isinstance(other, Tag):
            return Tag(self.time + other.time, self.microstep + other.microstep)
        else:
            raise TypeError("Unsupported operand type")

    def __eq__(self, other):
        if isinstance(other, Tag):
            return self.time == other.time and self.microstep == other.microstep
        else:
            return False

    def __lt__(self, other):
        if isinstance(other, Tag):
            if other is FOREVER:  # FOREVER is greater than anything
                return True
            if self is FOREVER:  # FOREVER is greater than anything
                return False
            return self.time < other.time or (self.time == other.time and self.microstep < other.microstep)
        else:
            raise TypeError("Unsupported operand type")


class Enclave:
    def __init__(self, name, df, availability, consistency):
        self.name = name
        self.trace = df
        self.currentTag = Tag()
        self.NET = 0
        self.idx = 0
        self.availbility = availability
        self.consistency = consistency
    def next_event_tag(self):
        return self.trace[self.idx][1]
    
    def execute_current_reaction(self):
        self.trace[self.idx]


    def _advance(self):
        self.idx += 1
        self.NET = self.next_event_tag().time

    def get_tag(self, consistency):
        return Tag(self.NET + consistency, 0)
    



# Define FOREVER as a specific instance of Tag that's greater than any other Tag
FOREVER = Tag(float('inf'), float('inf'))