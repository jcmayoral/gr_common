import matplotlib.pyplot as plt
import numpy as np
import rospy
import copy
import sys
import os

def read_file(filename):
    with open(filename,"r") as f:
        data = f.readlines()
    f = lambda data : [float(d) for d in data.rstrip().split(" ") ]
    data = map(f, data)
    return np.asarray(list(data))

def read_file2(filename):
    with open(filename,"r") as f:
        data = f.readlines()
    f = lambda data : [d for d in data.rstrip().split(" ") ]
    data = map(f, data)
    return np.asarray(list(data))

def mean_distance(person1, person2):
    distances = list()
    for d in person1:
        distances.append(d)
    for d in person2:
        distances.append(d)
    print("Mean Collision Distance " , np.mean(np.asarray(distances)))

def plot_coordinates(data, title, filename, new = True):
    if new:
        plt.figure()
        plt.title(title)
    for d in data:
        plt.scatter(d[:,0], d[:,1])
    plt.savefig(filename)

#check
def plot_coordinates2(data, vxs, title, filename, save=True):
    #if new:
    plt.figure()
    plt.title(title)
    plt.scatter(data[:,0], data[:,1], s = 20, c=vxs/np.max(vxs), alpha=0.5, cmap='jet')
    plt.colorbar()
    if save:
        plt.savefig(filename)


def time_between_collision(person1, person2,start_time, stop_time):
    time_between_collision = list()
    #tiempo desde iniicio a primera colision
    time_between_collision.append(person1[0] - start_time)

    for data in (person1, person2):    
        t0 = data[0]

        for i in range(1, len(data)):
            time_between_collision.append(data[i] - t0)
            t0 = copy.copy(data[i])

    #tiempo desde ultima colision a final
    time_between_collision.append(stop_time - person2[-1])

    print ("Mean Time between collisions " , np.mean(time_between_collision))

def mean_execution_time(stops, starts):
    execution_times = list()
    for s, st in zip(starts, stops):
        execution_times.append(st-s)
    print("Mean Execution Time " , np.mean(np.asarray(execution_times)))

def average_speed(vxs, vys, vzs):
    print("Mean speed {} {} {}".format(np.mean(vxs),np.mean(vys),np.mean(vzs)))

def filter_collisions(odom, collisions):
    collision_time = collisions[:,1]
    odom_time = odom[:,1]
    vxs = odom[:,8]
    authenticity = list()

    average_speed = list()

    for collision in collisions:
        collision_time = collision[1]
        closest_time = 1000000
        idx = -1
        vx = -1
        #best index
        for o in range(len(odom)):
            odom_time = odom[o,1]
            velx = odom[o,8]
            if np.fabs(odom_time - collision_time) < closest_time:
                idx = o
                vx = velx
                closest_time = np.fabs(odom_time - collision_time)
        authenticity.append(np.fabs(vx)>0.1)
        if np.fabs(vx)>0.1:
            average_speed.append(vx)

    assert len(collisions) == len(authenticity)

    authentic_collisions = list()

    for i in range(len(authenticity)):
        if (authenticity[i]):
            authentic_collisions.append(collisions[i])

    print ("Avergae speed on collision ", np.mean(np.asarray(average_speed)))
    print(len(collisions), len(authentic_collisions))
    return np.asarray(authentic_collisions)

def process_states(states, filename):
    acc = np.zeros((5,5))
    map_idx = dict({'Lethal': 0, 'Danger': 1, 'Warning': 2, 'Safe': 3, 'Unknown': 4})

    for i in range(len(states)):
        ind1 = map_idx[states[i,1]]
        ind2 = map_idx[states[i,2]]    
        acc[ind1, ind2] += 1
    plt.matshow(acc)
    plt.colorbar()
    #plt.show()
    plt.savefig(filename)


if __name__== "__main__":
    if len(sys.argv) != 2:
        sys.exit()
    experiment = sys.argv[1]
    person1_raw = read_file("{}/collision.txt".format(experiment))

    person2_raw = read_file("{}/collision_0.txt".format(experiment))
    start = read_file("{}/start.txt".format(experiment))[:,1]
    stop = read_file("{}/stop.txt".format(experiment))[:,1]    
    odom = read_file("{}/odom.txt".format(experiment))

    if os.path.exists("{}/state_transitions.txt".format(experiment)):
        states = read_file2("{}/state_transitions.txt".format(experiment))
        process_states(states,"{}/conf_matrix.png".format(experiment))

    person1 = filter_collisions(odom,person1_raw)
    person2 = filter_collisions(odom,person2_raw)


    #average speed
    average_speed(odom[:,-3], odom[:,-2], odom[:,-1])
    #odom pose
    plot_coordinates2(odom[:,2:4], odom[:,8], "robot_path", "{}/robot_path.png".format(experiment))

    #Execution Time
    mean_execution_time(stop, start)
    #run_id 0
    #time 1
    time_between_collision(person1[:,1], person2[:,1], start[0], stop[-1])
    #odom pose 2 3
    plot_coordinates((person1[:,2:4],person2[:,2:4]), "Collision in Map coordinates", "{}/odom_coordinates.png".format(experiment))
    #base_link pose 4 5
    plot_coordinates((person1[:,4:6],person2[:,4:6]), "Collision in Relative coordinates", "{}/local_coordinates.png".format(experiment))
    #distance 6
    mean_distance(person1[:,-1], person2[:,-1])

    plot_coordinates2(odom[:,2:4], odom[:,8], "Collisions and Robot Path","", save=False)
    plot_coordinates((person1[:,2:4],person2[:,2:4]), "", "{}/robot_collision.png".format(experiment), new=False)
