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
watch_intensity = 200 # the change that is nessessary for the watchman to get alarmed
watch_dist = 1000 # radius in which the watchman watches

def get_angle(angle_list, distance_list, max_min="max"):
    """returns the angle to the max/min distance in the distance_list from the angle_list"""
    if max_min=="max":
        dist = max(distance_list)
    else:
        dist = min(distance_list)
    dist_index = distance_list.index(dist)
    angle = angle_list[dist_index]
    return angle, dist

def get_drive_angle(sensor_angle):
    """Retruns the angle, the gpg will have to turn to face the given sensor_angle"""
    drive_angle = (sensor_angle-91)*(-1.05)
    return drive_angle


gpg.drive_cm(100) # so the gpg starts his watch in the front

while True:
    print("Initierung") # messures the field in its inital apperence
    angle_list_bef = []
    distance_list_bef = []
    for angle in range(min_angle, max_angle, scan_angle_inc):
        servo.rotate_servo(angle)
        time.sleep(sleep_time)
        dist = distance_sensor.read_mm()
        angle_list_bef.append(angle)
        distance_list_bef.append(dist)
        time.sleep(0.2)

    new_start = False

    while True:
        i = 0
        angle_list = []
        distance_list = []
        if new_start:
            break
        for angle in range(min_angle, max_angle, scan_angle_inc):
            servo.rotate_servo(angle)
            time.sleep(sleep_time)
            dist = distance_sensor.read_mm()
            if distance_list_bef[i] - dist >= watch_intensity and dist <= watch_dist: # if there is something closer than before its considered an enemy and drive there
                gpg.open_eyes()
                gpg.orbit(get_drive_angle(angle), 0)
                gpg.drive_cm((dist/10)-5) # drive the distance but don't crash the enemy
                gpg.close_eyes()
                new_start = True
                break
            time.sleep(0.1)
            i = i + 1
        time.sleep(0.2)