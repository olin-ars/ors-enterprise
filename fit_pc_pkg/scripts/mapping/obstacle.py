# from bounds import MappingBounds


class MappingObstacle:
    """ Usage: If representing point object, set both x- and y-components to same value."""

    def __init__(self, bounds):
        self.bounds = bounds

    # def __init__(self, x, y):
    #     self.bounds = MappingBounds(x, x, y, y)
