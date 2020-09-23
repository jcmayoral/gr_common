
#HACK
distance_sensor_vehicle = 10

#average distance between vehicle and obstacle
def SM1(persons):
    #print type(persons)
    return sum(persons)/len(persons)
#min distance between vehicle and obstacle
def SM2(persons):
    return min(persons)
#min distance between vehicle and obstacle for each sensor
def SM3(persons):
    print ("TODO ADD -> Use MAX instead")
    return max(persons)
