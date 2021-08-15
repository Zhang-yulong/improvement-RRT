class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class obstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
