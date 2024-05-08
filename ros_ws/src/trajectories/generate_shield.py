import numpy as np
import sys

"""
Generates trajectories for single shield, double shield, and triple shield. 
These trajectories move in the y-z plane.

For double shield and triple shield, the drones must be at the following positions,
denoted by (x,y,z) coordinates, where the (xi,yi,zi) represent the initial coordinates
of the drone, presumingly in a straight line in y-plane at a height of z=1 m, 
and r is the radius of the circle:

Left (-y direction): (xi, yi-0.25, zi)
Center (for triple shield only): (xi, yi+0.5, zi+r)
Right (+y direction): (xi, yi+1.25, zi)


"""


def generate_time_position_pairs(t, rot):
    #generate time-position pairs for spiral trajectory
    #rot represents trajectory rotation over x-axis
    
    time_interval = 0.1
    time_position_pairs = []
    t_curr = 0

    for i in range (int(t//time_interval)):
        r = .5 #max height is 2 m, drones are at 1m at center
        y = r * np.cos(t_curr + rot) #rotation may play in differently
        z = r * np.sin(t_curr + rot)
        x = 0 #circle in the y-z plane
        time_position_pairs.append([t_curr, x, y, z])
        t_curr += time_interval

    return time_position_pairs

def write_pairs_to_csv(t):
    #generate time-position pairs for single shield
    pairs1 = generate_time_position_pairs(t, rot = 0)

    #write time-position pairs to csv file
    with open('single_shield.csv', 'w') as f:
        for pair in pairs1:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))

    double_pairs1 = generate_time_position_pairs(t, rot = np.pi)
    double_pairs2 = generate_time_position_pairs(t, rot = 0)
    
    with open('double_shield_left.csv', 'w') as f:
        #left represents the drone in the (-y) direction
        for pair in double_pairs1: 
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
    
    with open('double_shield_right.csv', 'w') as f:
        #right represents the drone in the (+y) direction
        for pair in double_pairs2:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
    
    triple_pairs1 = generate_time_position_pairs(t, rot = np.pi)
    triple_pairs2 = generate_time_position_pairs(t, rot = (np.pi/2))
    triple_pairs3 = generate_time_position_pairs(t, rot = 0)

    with open('triple_shield_left.csv', 'w') as f:
        #left represents the drone in the (-y) direction
        for pair in triple_pairs1: 
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
    
    with open('triple_shield_center.csv', 'w') as f:
        #center drone
        for pair in triple_pairs2:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
    
    with open('triple_shield_right.csv', 'w') as f:
        #right represents the drone in the (+y) direction
        for pair in triple_pairs3:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
        

def main():
    t = 2 * np.pi
    write_pairs_to_csv(t)
    
if __name__ == "__main__":
    main()