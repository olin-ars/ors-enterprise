#! /usr/bin/python

import simulator_main as sim
import time

if __name__ == '__main__':
    model = sim.WorldModel(2, 0)  # initial windspeed, windheading
    running = True
    while running:
        model.update_model()
        time.sleep(.1)
        print model.boat1.posStr()
