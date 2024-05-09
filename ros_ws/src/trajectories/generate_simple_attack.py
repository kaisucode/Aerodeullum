import numpy as np
    
def generate_time_position_pairs():
    #generate time-position pairs for simple forward-backward attack
    time_interval = 0.1
    time_position_pairs = []
    t_curr = 0


    #backwards motion
    time_forwards = 5
    for i in range (int(time_forwards//time_interval)+1):
        x = 0.1*(i+1)
        time_position_pairs.append([t_curr, x, 0, 0])
        t_curr += time_interval

    time_backwards = 7
    time_scale = 5/7 * 0.1
    for i in range (int((time_backwards//time_interval))+1):
        x_curr = x - (time_scale)*(i+1)
        time_position_pairs.append([t_curr, x_curr, 0, 0])
        t_curr += time_interval
    
    return time_position_pairs

def write_pairs_to_csv():
    pairs = generate_time_position_pairs()

    #write time-position pairs to csv file
    with open('simple_attack.csv', 'w') as f:
        for pair in pairs: #spiral shrinks instead of grows
            f.write("%f, %f, %f, %f\n" % (pair[0], pair[1], pair[2], pair[3]))

def main():
    write_pairs_to_csv()
    
if __name__ == "__main__":
    main()