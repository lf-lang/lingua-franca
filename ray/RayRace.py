@ray.remote
class Incrementor():
    def __init__(self):
        # Initialize the environment.
        self.count = 0;
    def double(self):
        self.count *= 2;
        return self.count;
    def increment(self):
        self.count += 1;
        return self.count;

@ray.remote
class Doubler():
    def double(self, incrementor):
        return incrementor.double.remote();
        
@ray.remote
def test():
    incrementor = Incrementor.remote();
    doubler = Doubler.remote();
    first = doubler.double.remote(incrementor);
    second = incrementor.increment.remote();
    return ray.get(first) + ray.get(second);
    
    

What is the answer?
Looks to me like it could be 1 or 3, depending on the order in which the calls to double() and increment() occur.

The Ray paper says that there is an implicit dependency between double() and increment() because they both (potentially) alter the state. But the direction of that dependency seems to be determined by the order in which they are called. So this would be better described as a mutual exclusion, or mutual atomicity, than as a dependency.