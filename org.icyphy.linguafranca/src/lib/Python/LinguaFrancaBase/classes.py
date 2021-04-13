# A class used to generate variables with dots in their name
# Taken from tinyurl.com/y4chuth3
class Make:
    def __getattr__(self, name):
        self.__dict__[name] = Make()
        return self.__dict__[name]