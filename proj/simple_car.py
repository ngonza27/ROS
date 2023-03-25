#!/usr/bin/env python

#This file goes inside /home/carla/Desktop/catkin_ws/src/av_carla_examples/scripts
import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import random
import time
import numpy as np
import cv2

IMG_WIDTH = 640
IMG_HEIGHT = 480

actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    ## ADD A VEHICLE
    tesla_model3_bp = blueprint_library.filter('model3')[0]
    tesla_model3_bp.set_attribute('role_name', 'ego_vehicle')
    print(tesla_model3_bp)
    #spawn_point = random.choice(world.get_map().get_spawn_points())
    spawn_point = carla.Transform(carla.Location(x=-7.46, y=202.164, z=3.00), carla.Rotation(pitch=0.0, yaw =90.0, roll=0.0))
    vehicle = world.spawn_actor(tesla_model3_bp, spawn_point)
    control_vehicle = carla.VehicleControl()
    vehicle.apply_control(control_vehicle)
    actor_list.append(vehicle)  


    # sleep for 10 seconds, then finish:
    time.sleep(100)

finally:

    print('Cleaning up actors...')
    for actor in actor_list:
        actor.destroy()
    print('Done, Actors cleaned-up successfully!')


