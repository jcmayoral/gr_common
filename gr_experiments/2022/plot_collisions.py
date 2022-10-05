import matplotlib.pyplot as plt
import numpy as np
import rospy

def read_file(filename):
    with open(filename,"r") as f:
        data = f.readlines()
    f = lambda data : [float(d) for d in data.rstrip().split(" ") ]
    data = map(f, data)
    return np.asarray(list(data))

def plot_distance(data, nboxes=10):
    names = np.arange(0,10,1)
    nmax = np.max(data)
    fd = [int(i/nmax*nboxes) for i in data]
    unique, counts = np.unique(fd, return_counts=True)
    data = dict(zip(unique, counts))
    print(data.keys())
    plt.figure()
    plt.bar(data.keys(), data.values())
    plt.show()

def time_between_collision(data):
    flip_data = list(np.flip(data))
    time1 = flip_data.pop()
    time2 = flip_data.pop()

    time_between_collision = list()
    time_between_collision.append(time2-time1)

    while (len(flip_data)>0):
        time1 = flip_data.pop()
        rt2 = rospy.Time.from_sec(time2).to_sec()
        rt1 = rospy.Time.from_sec(time1).to_sec()
        time_between_collision.append((rt2-rt1)/1000000000)
        time2, time1 = time2, time1

    print (time_between_collision)


if __name__== "__main__":
    data = read_file("benchmark/collision.txt")
    #run_id 0
    #time 1
    time_between_collision(data[:,1])
    #odom pose 2 3
    #base_link pose 4 5
    #distance 6
    plot_distance(data[:,1])