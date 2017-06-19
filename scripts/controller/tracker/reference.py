
class Reference(object):
    def __init__(self,pos,speed):
        self.pos = pos
        self.speed = speed

    def __eq__(self, other):
        if isinstance(other, Reference):
            return self.pos == other.pos and self.speed == other.speed
        return NotImplemented

    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result
