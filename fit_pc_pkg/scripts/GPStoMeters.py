import math

class SetHome():
    """ Takes in a home location and has a method to convert from lat lon
    to m relative to home """
    def __init__(self, home=(44.220954, -76.485424)):
        """ sets home location and scaling factors """
        self.initScalars(home[0])
        self.home = home
    def transformLocation(self, lat, longitude):
        """ converts GPS location to location in meters relative to home """
        return ((lat - self.home[0]) * self.latscalar,
                (longitude - self.home[1]) * self.longscalar)

    def initScalars(self, lat):
        """ finds conversions from lat lon to m at given home location """
        earth_circumference = 40.075 * 10**6  # meters
        self.latscalar = earth_circumference / 360  # meters per degree

        longdistance = math.cos(math.radians(lat)) * earth_circumference  # m

        self.longscalar = longdistance / 360  # meters per degree
