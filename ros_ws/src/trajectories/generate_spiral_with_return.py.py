import numpy as np
import math

time_interval = 0.1
time_position_pairs = []

def generate_time_position_pairs_start():
    #generate time-position pairs to move in +x direction 0.5m before starting spiral trajectory
    t_curr = 0
    x_init = 0

    for i in range(int(1//time_interval) + 1):
        x = x_init + 0.05*(i+1)
        time_position_pairs.append([t_curr, x, x, 0])
        t_curr += time_interval
    return t_curr

def generate_time_position_pairs(t_init):
    #generate time-position pairs for spiral trajectory
    
    time_interval = 0.1
    t_init = t_init
    t_curr = 0
    t = 2 * np.pi
    x_init = time_position_pairs[-1][1]

    for i in range (int(t//time_interval)):
        r=.5 * (1.25** -t_curr) #spiral shrinks instead of grows
        y = r * np.cos(2*t_curr)
        z = r * np.sin(2*t_curr)
        x = t_curr/3 #can stretch or shrink to fit length of attack lane
        time_position_pairs.append([t_init + t_curr, x_init + x, y, z])
        t_curr += time_interval

    return t_init + t_curr

def generate_time_position_pairs_return(t_init):
    #generate time-position pairs for return to initial position after spiral trajectory
    t_curr = t_init 
    x_init = time_position_pairs[-1][1]
    y_init = time_position_pairs[-1][2]
    z_init = time_position_pairs[-1][3]

    #move in +z direction 0.5m
    for i in range (int(1//time_interval)):
        z_curr = z_init + 0.05*(i+1)
        time_position_pairs.append([t_curr, x_init, y_init, z_curr])
        t_curr += time_interval
    
    #move in +y direction 1m
    for i in range (int(2//time_interval)):
        y_curr = y_init - 0.05*(i+1)
        time_position_pairs.append([t_curr, x_init, y_curr, z_curr])
        t_curr += time_interval

    #move in -x direction x_init m
    for i in range (2*int(x_init//time_interval)):
        x_curr = x_init - 0.05*(i+1)
        time_position_pairs.append([t_curr, x_curr, y_curr, z_curr])
        t_curr += time_interval

    #move in -y direction y_curr m
    for i in range (2*int(y_curr//time_interval)):
        y = y_curr - 0.05*(i+1)
        time_position_pairs.append([t_curr, x_curr, y, z_curr])
        t_curr += time_interval
    
    #move in -z direction z_curr m
    for i in range (int(2*z_curr//time_interval)):
        z = z_curr - 0.05*(i+1)
        time_position_pairs.append([t_curr, x_curr, y, z])
        t_curr += time_interval

    time_position_pairs.append([t_curr, 0, 0, 0]) #final position


def write_pairs_to_csv():
    time_to_start = generate_time_position_pairs_start()
    spiral_time = generate_time_position_pairs(time_to_start)
    generate_time_position_pairs_return(spiral_time)

    #write time-position pairs to csv file
    with open('spiral2.csv', 'w') as f:
        for pair in time_position_pairs: #spiral shrinks instead of grows
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))

def main():
    write_pairs_to_csv()
    
if __name__ == "__main__":
    main()