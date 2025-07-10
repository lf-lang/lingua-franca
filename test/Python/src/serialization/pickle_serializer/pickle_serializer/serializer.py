import pickle

class Serializer():
    def serialize(self, obj)->bytes:
        return pickle.dumps(obj)
    def deserialize(self, message:bytes):
        return pickle.loads(message)