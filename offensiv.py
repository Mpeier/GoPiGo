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


def get_angle(angle_list, distance_list, max_min="max"):
    """returns the angle to the max/min distance in the distance_list from the angle_list"""
    if max_min=="max":
        dist = max(distance_list)
    elif max_min == "min":
        dist = min(distance_list)
    dist_index = distance_list.index(dist)
    angle = angle_list[dist_index]
    return angle, dist

def get_drive_angle(sensor_angle):
    """Retruns the angle, the gpg will have to turn to face the given sensor_angle"""
    drive_angle = (sensor_angle-91)*(-1.07)
    return drive_angle

def avoid_turn_around(current_orientation, angle_list, distance_list):
    """Takes current orientation and modifies the distances and angles list so the gpg won't turn 180 degrees"""
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
    """Will turn the gpg in the best scanned direction without it ever fully turning around"""
    angle_list, distance_list = avoid_turn_around(gpg_angle, angle_list, distance_list)
    print(angle_list)
    print(distance_list)
    #if get_angle(angle_list, distance_list, max_min="min") < min_dist:

    sensor_direction = get_angle(angle_list, distance_list)[0]
    print(sensor_direction)
    gpg_direction = get_drive_angle(sensor_direction)
    print(gpg_direction)
    gpg.orbit(gpg_direction, turn_radius)
    travel_way.append("gpg.orbit," + str(gpg_direction) + "," + str(turn_radius))
    return gpg_direction

def end_detection(distance_list, max_dist=1000):
    """Checks if there is nothing left and right of th gpg und assums that it's done then"""
    if distance_list[0] >= max_dist and distance_list[-1] >= max_dist:
        #vllt augen öffnen
        return True
    else:
        return False

def enemy_detection(angle_list, distance_list):
    """Checks for small objects in the scanner radius and returns a list with the angles where they were found"""
    pot_enemy_angles = []
    pot_enemy_distances = []
    for i in range(len(distance_list)-2):
        distance1 = distance_list[i]
        distance2 = distance_list[i+1] + 100 # 10cm tollerance so there are no false positives
        distance3 = distance_list[i+2]
        if distance1 > distance2 and distance2 < distance3:
            pot_enemy_angles.append(angle_list[i+1])
            pot_enemy_distances.append(distance_list[i+1])

    return pot_enemy_angles, pot_enemy_distances

def dodge(enemy_angles, enemy_distances, right_bound=70, left_bound=110, drive_dist=20):
    """Takes potential enemy lists and trys to avoid the closest one"""
    closest_angle = get_angle(enemy_angles, enemy_distances, max_min="min")[0]
    if closest_angle > left_bound:
        gpg.orbit(45)
        gpg.drive_cm(drive_dist)
        gpg.orbit(-45)
    if closest_angle < right_bound:
        gpg.orbit(-45)
        gpg.drive_cm(drive_dist)
        gpg.orbit(45)
    if closest_angle > right_bound and closest_angle < left_bound:
        gpg.orbit(-90)
        gpg.drive_cm(drive_dist)
        gpg.orbit(90)


while True:
    angle_list = []
    distance_list = []
    dodged = False
    for angle in range(min_angle, max_angle, scan_angle_inc):
        servo.rotate_servo(angle)
        time.sleep(sleep_time)
        dist = distance_sensor.read_mm()
        angle_list.append(angle)
        distance_list.append(dist)
        time.sleep(0.1)

    if get_angle(angle_list, distance_list, max_min="min")[1] < 500:
        pot_enemy_angles, pot_enemy_distances = enemy_detection(angle_list, distance_list)
        if len(pot_enemy_distances) > 0:
            dodge(pot_enemy_angles, pot_enemy_distances)
            dodged = True

    if end_detection(distance_list):
        gpg.open_eyes()
        time.sleep(1)
        gpg.close_eyes()
        break
    if dodged == False: # if the gpg didn't dodge he shall drive normally to its goals
        gpg_angle = gpg_angle + turn_to_direction(angle_list, distance_list)
        print(gpg_angle)
        distance = (get_angle(angle_list, distance_list)[1])/10-20
        if distance > max_dist:
            distance = 100
        gpg.drive_cm(distance)
        travel_way.append("gpg.drive_cm(" + str(distance))

