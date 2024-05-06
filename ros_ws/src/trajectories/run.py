from blocklyTranslations import *
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np

def drone_takeoff(all_cfs):
    for i in range(all_cfs):
        takeoff(groupState=i, height=1, duration=2)

def drone_land(all_cfs):
    for i in range(all_cfs):
        land(groupState=i, height=0, duration=2)

def single_attack(drone_id: int):
    #move in a straight line or spiral trajectory
    pass

def triple_attack(drone_ids, helix_path):
    #move in helix trajectory
    helix_traj = Trajectory()
    helix_traj.loadcsv(helix_path)
    cf1 = drone_ids[0]
    cf2 = drone_ids[1]
    cf3 = drone_ids[2]

    TIMESCALE = 1.0

    for cf in drone_ids:
        cf.uploadTrajectory(0,0,helix_traj)
    
    #allcfs should have the same x_pos, add their coordinates from the initial pos to the coordinate they need to be in
    cf1_pos = np.array(cf1.initialPosition) + np.array([]) #fill coords here
    cf2_pos = np.array(cf2.initialPosition) + np.array([])
    cf3_pos = np.array(cf2.initialPosition) + np.array([])

    cf1.goTo(cf1_pos, 0, 2)
    cf2.goTo(cf2_pos, 0, 2)
    cf3.goTo(cf3_pos, 0, 2)

    for cf in drone_ids:
        cf.startTrajectory(0, timescale=TIMESCALE)
    
    #does timeHelper need to sleep? how can we do that for only some of the drones
    #timeHelper.sleep(helix_traj.duration * TIMESCALE + 2)
    
    #send cfs through cooldown lane and return to initial pos
    for cf in drone_ids:
        cooldown_pos
        cf.goTo(cf1.currentPosition)
    
    pass

def shield(drone_ids):
    pass





 


