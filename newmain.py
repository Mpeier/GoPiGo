from __future__ import print_function
from __future__ import division
from builtins import input

import time
import easygopigo3 as easy
import sys
import atexit


#Initialisierung

gpg = easy.EasyGoPiGo3(use_mutex=True)
atexit.register(gpg.stop)
servo = gpg.init_servo()
distance_sensor = easy.DistanceSensor('I2C', gpg, use_mutex=True)
travel_way = [] #Liste mit allen Manöfern die der GoPiGo auf seinem Weg gemacht hat

# We wanted to use a logfile for the waypoints, but rasberryos didn't allow it
#filename = "travel_log.txt"
#open(filename, 'w').close() #clear log file from last run
#file = open(filename, 'a') # open logfile in append mode

scan_angle_inc = 30 # Scanner angle Increment
gpg_angle = 0 # View direction of gpg
min_dist = 80 # min Distanz to avoid collisons

min_angle = 0 # min scanner position
max_angle = 181 # max scanner position
sleep_time = 0.5 # time to do one scan
max_dist = 150 # max distance to end the program

def get_angle_distance(angle_list, distance_list, max_min="max"):
    """returns the angle or distance to the max/min distance in the distance_list from the angle_list"""
    if max_min=="max":
        dist = max(distance_list)
    else:
        dist = min(distance_list)
    dist_index = distance_list.index(dist)
    angle = angle_list[dist_index]
    return angle, dist

def get_drive_angle(sensor_angle):
    """Returns the angle, the gpg will have to turn to face the given sensor_angle (0 to 180 to -90 to 90)"""
    drive_angle = (sensor_angle-91)*(-1.07)
    return drive_angle

def avoid_turn_around(current_orientation, angle_list, distance_list):
    """Takes current orientation and modifies the distances list so the gpg won't turn 180 degrees, unless there is no other way"""
    if current_orientation < -90:
        for i in range(len(distance_list)):
            if distance_list[i] > min_dist and angle_list[i] > 90:
                distance_list[i] = min_dist + 1
    if current_orientation > 90:
        for i in range(len(distance_list)):
            if distance_list[i] > min_dist and angle_list[i] < 90:
                distance_list[i] = min_dist + 1
    return angle_list, distance_list

def turn_to_direction(angle_list, distance_list, turn_radius=5):
    """Will turn the gpg in the best scanned direction without it fully turning around"""
    angle_list, distance_list = avoid_turn_around(gpg_angle, angle_list, distance_list)
    sensor_direction = get_angle_distance(angle_list, distance_list)[0]
    gpg_direction = get_drive_angle(sensor_direction)
    gpg.orbit(gpg_direction, turn_radius)
    travel_way.append("gpg.orbit," + str(gpg_direction) + "," + str(turn_radius)) # write move to travel log
    return gpg_direction

def end_detection(distance_list, max_dist=1000):
    """Checks if there is nothing left and right of th gpg und assums that it's done then"""
    if distance_list[0] >= max_dist and distance_list[-1] >= max_dist:
        return True
    else:
        return False

gpg.close_eyes()

while True:
    #We wanted to put all the following in a methode, but it didn't work with the exact same code
    angle_list = []
    distance_list = []
    for angle in range(min_angle, max_angle, scan_angle_inc):
        servo.rotate_servo(angle)
        time.sleep(sleep_time)
        dist = distance_sensor.read_mm()
        angle_list.append(angle)
        distance_list.append(dist)
        time.sleep(0.1)

    if end_detection(distance_list):
        gpg.open_eyes()
        break

    gpg_angle = gpg_angle + turn_to_direction(angle_list, distance_list) # the gpgs direction is its current direction + the direction it turned to
    distance = (get_angle_distance(angle_list, distance_list)[1])/10-20 # drives as far as possible with a savety distance
    if distance > max_dist:
        distance = 100 # drive max 1 m
    gpg.drive_cm(distance)
    travel_way.append("gpg.drive_cm," + str(distance)) # write move to travel log

print(travel_way)
if input("Wiederholen (Y/N)? ") == "Y":
    gpg.close_eyes()
    for command in travel_way:
        if command.split(",")[0] == "gpg.drive_cm":
            gpg.drive_cm(float(command.split(",")[1]))
        if command.split(",")[0] == "gpg.orbit":
            gpg.orbit(float(command.split(",")[1]), float(command.split(",")[2]))
        time.sleep(1)
    gpg.open_eyes()
    print("Done!")
else:
    print("Done!")
