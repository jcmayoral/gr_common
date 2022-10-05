import matplotlib.pyplot as plt
import numpy as np

def read_file(filename):
    with open(filename,"r") as f:
        data = f.readlines()
    f = lambda data : [float(d) for d in data.rstrip().split(" ") ]
    data = map(f, data)
    return np.asarray(list(data))


if __name__== "__main__":
    data = read_file("test_results/collision.txt")
    print (data[:,1].shape, "A")
    #run_id 0
    #time 1
    #odom pose 2 3
    #base_link pose 4 5
    #distance 6
    plt.figure()
    plt.bar(data[:,-1], 10, 0.1)
    plt.show()