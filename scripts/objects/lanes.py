from .point import Point


class Lanes:
    def __init__(self, right=Point(0, 0), right_ahead=Point(0, 0), left=Point(0, 0), left_ahead=Point(0, 0)):
        self.right = right
        self.left = left

        self.right_ahead = right_ahead
        self.left_ahead = left_ahead

    def are_visible(self):
        return self.right.x or self.right.y or self.left.x or self.left.y

    def are_both_visible(self):
        return (self.right.x or self.right.y) and (self.left.x or self.left.y)

    def is_right_visible(self):
        return self.right.x or self.right.y

    def is_left_visible(self):
        return self.left.x or self.left.y
