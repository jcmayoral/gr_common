import matplotlib.pyplot as plt
import numpy as np

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

if __name__== "__main__":
    data = read_file("test_results/collision.txt")
    print (data[:,1].shape, "A")
    #run_id 0
    #time 1
    #odom pose 2 3
    #base_link pose 4 5
    #distance 6
    plot_distance(data[:,1])