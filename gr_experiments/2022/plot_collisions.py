import matplotlib.pyplot as plt
import numpy as np
import rospy
import copy
import sys

def read_file(filename):
    with open(filename,"r") as f:
        data = f.readlines()
    f = lambda data : [float(d) for d in data.rstrip().split(" ") ]
    data = map(f, data)
    return np.asarray(list(data))

def mean_distance(person1, person2):
    distances = list()
    for d in person1:
        distances.append(d)
    for d in person2:
        distances.append(d)
    print("Mean Collision Distance " , np.mean(np.asarray(distances)))

def plot_coordinates(person1, person2, title, filename):
    print (person1)
    plt.figure()
    plt.title(title)
    for data in (person1, person2):
        plt.scatter(data[:,0], data[:,1])
    plt.savefig(filename)

def time_between_collision(person1, person2):
    time_between_collision = list()

    for data in (person1, person2):    
        t0 = data[0]

        for i in range(1, len(data)):
            time_between_collision.append(data[i] - t0)
            to = copy.copy(data[i])

    print ("Mean Time between collisions " , np.mean(time_between_collision))

def mean_execution_time(stops, starts):
    execution_times = list()
    for s, st in zip(starts, stops):
        execution_times.append(st-s)

    print("Mean Execution Time " , np.mean(np.asarray(execution_times)))


if __name__== "__main__":
    if len(sys.argv) != 2:
        sys.exit()
    experiment = sys.argv[1]
    person1 = read_file("{}/collision.txt".format(experiment))
    person2 = read_file("{}/collision_0.txt".format(experiment))
    start = read_file("{}/start.txt".format(experiment))[:,1]
    stop = read_file("{}/stop.txt".format(experiment))[:,1]    
    #Execution Time
    mean_execution_time(stop, start)

    #run_id 0
    #time 1
    time_between_collision(person1[:,1], person2[:,1])
    #odom pose 2 3
    plot_coordinates(person1[:,2:4],person2[:,2:4], "Collision in Map coordinates", "{}/odom_coordinates.png".format(experiment))
    #base_link pose 4 5
    plot_coordinates(person1[:,4:6],person2[:,4:6], "Collision in Relative coordinates", "{}/local_coordinates.png".format(experiment))
    #distance 6
    mean_distance(person1[:,-1], person2[:,-1])