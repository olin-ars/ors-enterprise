class MappingBounds:

    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

    """ Calculates if two squares intersect. Does not count just touching left
        or bottom edge as intersecting.
        Preconditions: For each bound, x1 < x2, y1 < y2 """
    @staticmethod
    def does_intersect(bounds1, bounds2):

        if (bounds1.x1 < bounds2.x2 and
                bounds1.x2 >= bounds2.x1 and
                bounds1.y1 < bounds2.y2 and
                bounds1.y2 >= bounds2.y1):
            return True
        return False
