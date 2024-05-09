#!/usr/bin/env python

import numpy as np
from pathlib import Path

from crazyflie_py import *
from crazyflie_py.uav_trajectory import Trajectory


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / "aero/p2_helix1.csv")
    traj2 = Trajectory()
    traj2.loadcsv(Path(__file__).parent / "aero/p2_helix2.csv")
    traj3 = Trajectory()
    traj3.loadcsv(Path(__file__).parent / "aero/p2_helix3.csv")
    traj4 = Trajectory()
    traj4.loadcsv(Path(__file__).parent / "aero/p2_simple.csv")
    traj5 = Trajectory()
    traj5.loadcsv(Path(__file__).parent / "aero/p2_familiar.csv")
    traj6 = Trajectory()
    traj6.loadcsv(Path(__file__).parent / "aero/p2_single_shield.csv")
    traj7 = Trajectory()
    traj7.loadcsv(Path(__file__).parent / "aero/p2_spiral.csv")
    traj8 = Trajectory()
    traj8.loadcsv(Path(__file__).parent / "aero/p2_straight.csv")
    trajectories = [traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8]
    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        for i in range(len(trajectories)):

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 4.0)
        timeHelper.sleep(4.5)
        for i in range(8):
            for cf in allcfs.crazyflies:
                cf.uploadTrajectory(i, 0, trajectories[i])
            timeHelper.sleep(0.25)
            allcfs.startTrajectory(i, timescale=TIMESCALE)
            timeHelper.sleep(trajectories[i].duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.0, duration=2.0)
        timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
