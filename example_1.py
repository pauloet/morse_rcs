#! /usr/bin/env python3.2
import sys
import rcs
from time import sleep
def main():


    r1r2_1 = rcs.RCS('robo1', 'robo2', model = "distance", distance_threshold = 15)
    r1r2_2 = rcs.RCS('robo1', 'robo2', model = "line_of_sight")
    r1r2_3 = rcs.RCS('robo1', 'robo2', model = "free_space_loss", freq = 750, free_space_threshold = 55)

    parameters = {'t1':10, 't2':20, 't3':30, 'dr0':4, 'dr1':3, 'dr2':2, 'dr3':1}
    r1r2_4 = rcs.RCS('robo1', 'robo2', model = 'plm', plm = parameters)

    try:
        while True:
            print("Distance Model: %i" %(r1r2_1.can_communicate()))    
            print("Line of Sight Model: %i" %(r1r2_2.can_communicate()))
            print("Free Space Loss Model: %i" %(r1r2_3.can_communicate()))
            print("Path Loss Map Model: %i" %(r1r2_4.can_communicate()))
            sleep(7)    # seconds
    except (KeyboardInterrupt, SystemExit):
        print(' --> Programm terminated.')
        raise     
    finally: 
        del r1r2_1
        del r1r2_2
        del r1r2_3
        del r1r2_4
        pass

if __name__ == "__main__":
    main()
