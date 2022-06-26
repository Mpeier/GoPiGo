from __future__ import print_function
from __future__ import division
from builtins import input

import time
import easygopigo3 as easy
import sys
import atexit


#Initialisierung
modus = ["labyrinth", "labyrinth_fast", "fb_offence", "fb_defence"]
mod = modus[0]

gpg = easy.EasyGoPiGo3(use_mutex=True)
atexit.register(gpg.stop)
servo = gpg.init_servo()
distance_sensor = easy.DistanceSensor('I2C', gpg, use_mutex=True)
travel_way = [] #Liste mit allen Manöfern die der GoPiGo auf seinem Weg gemacht hat

filename = "travel_log.txt"
open(filename, 'w').close() #clear log file from last run
file = open(filename, 'a') # open logfile in append mode

#Scanner angle Increment
scan_angle_inc = 30
#Ausrichtung des gpg
gpg_angle = 0
min_dist = 80 #min Distanz to avoid collishans

def distance_scan(increment=scan_angle_inc, min_angle=0, max_angle=180, sleep_time=0.5):
    """Scans the front of die gpg in given increments and returns two lists for the messured distances and their angles"""
    angle_list = []
    distance_list = []
    for angle in range(min_angle, max_angle, increment):
        servo.rotate_servo(angle)
        time.sleep(sleep_time)
        dist = distance_sensor.read_mm()
        angle_list.append(angle)
        distance_list.append(dist)
        time.sleep(0.1)
        return angle_list, distance_list

def get_angle(angle_list, distance_list, max_min="max"):
    """returns the angle to the max/min distance in the distance_list from the angle_list"""
    if max_min=="max":
        dist = max(distance_list)
    else:
        dist = min(distance_list)
    dist_index = distance_list.index(dist)
    angle = angle_list[dist_index]
    return angle

def get_drive_angle(sensor_angle):
    """Retruns the angle, the gpg will have to turn to face the given sensor_angle"""
    drive_angle = (sensor_angle-91)*(-1.07)
    return drive_angle

def avoid_turn_around(current_orientation, angle_list, distance_list):
    """Takes current orientation and modifies the distances and angles list so the gpg won't turn 180 degrees"""
    if current_orientation < -90:
        for i in range(len(distance_list)):
            if distance_list[i] > min_dist and angle_list[i] < 90:
                distance_list[i] = min_dist + 1
    if current_orientation > 90:
        for i in range(len(distance_list)):
            if distance_list[i] > min_dist and angle_list[i] > 90:
                distance_list[i] = min_dist + 1
    return angle_list, distance_list

def turn_to_direction(angle_list, distance_list, turn_radius=0):
    """Will turn the gpg in the best scanned direction without it ever fully turning around"""
    angle_list, distance_list = avoid_turn_around(gpg_angle, angle_list, distance_list)
    sensor_direction = get_angle(angle_list, distance_list)
    gpg_direction = get_drive_angle(sensor_direction)
    gpg.orbit(gpg_direction, turn_radius)
    travel_way.append("gpg.orbit," + str(gpg_direction) + "," + str(turn_radius))
    return gpg_direction

def end_detection(angle_list, distance_list):
    """Checks if there is nothing left and right of th gpg und assums that it's done then"""
    if distance_list[0] == 3000 and distance_list[-1] == 3000:
        #vllt augen öffnen
        return True
    else:
        return False


def enemy_detection():
    """Checks for small objects in the scanner radius and returns a list with the angles where they were found"""
    angle_list, distance_list = distance_scan()
    pot_enemy_angles = []
    pot_enemy_distances = []
    for i in range(len(distance_list)-2):
        distance1 = distance_list[i]
        distance2 = distance_list[i+1] + 100
        distance3 = distance_list[i+2]
        if distance1 > distance2 and distance2 < distance3:
            pot_enemy_angles.append(angle_list[i+1])
            pot_enemy_distances.append(distance_list[i+1])

    return pot_enemy_angles, pot_enemy_distances

def dodge(enemy_angles, enemy_distances, left_bound=70, right_bound=110, drive_dist=20):
    """Takes potential enemy lists and trys to avoid the closest one"""
    closes_angle = get_angle(enemy_angles, enemy_distances, max_min="min")
    if closes_angle < left_bound:
        gpg.orbit(45)
        gpg.drive_cm(drive_dist)
        gpg.orbit(-45)
    if closes_angle > right_bound:
        gpg.orbit(-45)
        gpg.drive_cm(drive_dist)
        gpg.orbit(45)
    if closes_angle < right_bound and closes_angle > left_bound:
        gpg.orbit(-90)
        gpg.drive_cm(drive_dist)
        gpg.orbit(90)

def watch_mode(sleep_time=5, turn_radius=5):
    """Takes the gpg in a static watch mode that allows him to detect (significant) chances in its scanning radius"""
    angle_list_inital, distance_list_inital = distance_scan(increment=15, max_angle=90) #für rechte Seite
    while True:
        angle_list, distance_list = distance_scan()
        for i in range(len(distance_list)):
            if round(distance_list[i],-2) != round(distance_list_inital[i],-2):
                gpg.orbit(get_drive_angle(angle_list[i]), turn_radius)
                gpg.drive_cm(distance_list[i])
                break
        time.sleep(sleep_time)

# Hauptprogramm

if mod == "labyrinth":
    while True:
        angle_list, distance_list = distance_scan()
        gpg_angle = gpg_angle + turn_to_direction()
        gpg.drive_cm(10)
        travel_way.append("gpg.drive_cm(10)")
        # write travel log to travel log file
        file.write(travel_way[-2] + '\n')
        file.write(travel_way[-1] + '\n')

        if end_detection():
            break

elif mod == "labyrinth_fast":
    with open(filename) as f:
        commands = f.read().splitlines()
    for command in commands:
        if command == "gpg.drive_cm(10)":
            gpg.drive_cm(10)
        if command.split(",")[0] == "gpg.orbit":
            gpg.orbit(command.split(",")[1], command.split(",")[2])
        time.sleep(1)

elif mod == "fb_offence":
    gpg.drive_cm(30)
    pot_enemy_angles, pot_enemy_distances = enemy_detection()
    dodge(pot_enemy_angles, pot_enemy_distances)

elif mod == "fb_defence":
    gpg.drive_cm(30) # initial drive to the startpoint for the watch
    watch_mode()