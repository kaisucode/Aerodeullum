import numpy as np
import math
    
def generate_time_position_pairs(t):
    #generate time-position pairs for spiral trajectory
    
    time_interval = 0.1
    time_position_pairs = []
    t_curr = 0


    for i in range (int(t//time_interval)):
        r=.5 * (1.25** -t_curr) #spiral shrinks instead of grows
        y = r * np.cos(2*t_curr)
        z = r * np.sin(2*t_curr)
        x = t_curr/3 #can stretch or shrink to fit length of attack lane
        time_position_pairs.append([t_curr, x, y, z])
        t_curr += time_interval

    return time_position_pairs


def write_pairs_to_csv(t):
    pairs = generate_time_position_pairs(t)

    #write time-position pairs to csv file
    with open('spiral.csv', 'w') as f:
        for pair in pairs: #spiral shrinks instead of grows
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))

def main():
    t = 2 * np.pi
    write_pairs_to_csv(t)
    
if __name__ == "__main__":
    main()