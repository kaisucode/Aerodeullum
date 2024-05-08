import numpy as np
    
def generate_time_position_pairs(t, rot):
    #generate time-position pairs for helix trajectory
    #parametric helix equation
    #spiral_function = k/t_curr #some decreasing function of t that does not reach 0 (avoid crash)
    r = 0.5 #change to spiral constant
    time_interval = 0.1
    time_position_pairs = []
    t_curr = 0
    for i in range (int(t//time_interval)):
        y = r * np.cos(t_curr + rot)
        z = r * np.sin(t_curr + rot)
        x = t_curr/3
        time_position_pairs.append([t_curr, x, y, z])
        t_curr += time_interval
    time_position_return_pairs = generate_time_position_pairs_return(time_position_pairs, t_curr)
    
    return time_position_return_pairs

def generate_time_position_pairs_return(pair: list[list[int]], t_curr):
    #generate time-position pairs for return to initial position after spiral trajectory
    t_curr = t_curr 
    time_interval = 0.1
    x_init = pair[-1][1]
    y_init = pair[-1][2]
    z_init = pair[-1][3]
    
    #move in +y direction 1 m
    for i in range (int(2//time_interval)):
        y_curr = y_init + 0.05*(i+1)
        pair.append([t_curr, x_init, y_curr, z_init])
        t_curr += time_interval
    
    y_2 = y_curr

    #move in -x direction x_init m
    for i in range (2*int(x_init//time_interval)):
        x_curr = x_init - 0.05*(i+1)
        pair.append([t_curr, x_curr, y_2, z_init])
        t_curr += time_interval


    #move in -y direction y_curr m
    for i in range(2*int(y_2//time_interval)):
        y = y_2 - 0.05*(i+1)
        pair.append([t_curr, x_curr, y, z_init])
        t_curr += time_interval

    pair.append([t_curr, 0, 0, 0]) #final position

    return pair


def write_pairs_to_csv(t):
    pairs1 = generate_time_position_pairs(t, 0)
    pairs2 = generate_time_position_pairs(t, 2*np.pi/3)
    pairs3 = generate_time_position_pairs(t, 4*np.pi/3)
    #write time-position pairs to csv file
    with open('helix1.csv', 'w') as f:
        for pair in pairs1:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
    with open('helix2.csv', 'w') as f:
        for pair in pairs2:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))
    with open('helix3.csv', 'w') as f:
        for pair in pairs3:
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))

def main():
    t = 4 * np.pi
    write_pairs_to_csv(t)
    
if __name__ == "__main__":
    main()