import numpy as np
    


def generate_time_position_pairs(t):
    #generate time-position pairs for familiar backwards stutter
    time_interval = 0.1
    time_position_pairs = []
    t_curr = 0

    #backwards motion
    time_backwards = t/3
    for i in range (int(time_backwards//time_interval)+1):
        x = -0.05*(i+1)
        time_position_pairs.append([t_curr, x, 0, 0])
        t_curr += time_interval

    time_forwards = 2 * (t/3)
    for i in range (int((time_forwards//time_interval))+1):
        x_curr = x + 0.025*(i+1)
        time_position_pairs.append([t_curr, x_curr, 0, 0])
        t_curr += time_interval
    
    return time_position_pairs

def write_pairs_to_csv(t):
    pairs = generate_time_position_pairs(t)

    #write time-position pairs to csv file
    with open('familiar.csv', 'w') as f:
        for pair in pairs: #spiral shrinks instead of grows
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))

def main():
    t = 3
    write_pairs_to_csv(t)
    
if __name__ == "__main__":
    main()