#!/usr/bin/env python3
import time 
import math
import re
import py_trees
import json
from py_trees.common import Status
from py_trees.decorators import Inverter
from py_trees import logging as log_tree
from py_trees.behaviour import Behaviour   
from py_trees.composites import Selector,Sequence,Parallel
import numpy as np
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative

# ready_for_next_batch = True
# positions in NED FRAME
# These Positions can be replaced by geotags that are generated




class RTLAction(Behaviour):
    def __init__(self, name,calledby,vehicle,state):
        super(RTLAction,self).__init__(name)
        # print('Battery is low returning to launch position')
        self.calledby = calledby
        self.vehicle = vehicle
        print('Called by ',calledby)
        self.data = None
        self.state = state
    def initialise(self):
        self.vehicle.parameters['RTL_ALT'] = 0
        self.vehicle.parameters['WPNAV_SPEED'] = 200
        self.vehicle.parameters['LAND_SPEED'] = 50
        self.vehicle.mode = VehicleMode('RTL')
        return super().initialise()

    def update(self):
         
        # if vehicle.mode.name != "GUIDED":
        #     print("WARNING: Not in GUIDED mode! Switching...")
        #     vehicle.mode = VehicleMode("GUIDED")
        #     time.sleep(1)
        with open(self.state["json_address"],"r") as knw:
            self.data = json.load(knw)
        print(f'NORTH: {self.vehicle.location.local_frame.north} , EAST: {self.vehicle.location.local_frame.east} , DOWN: {self.vehicle.location.local_frame.down} ')
        if self.get_dist(self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon,self.vehicle.location.global_frame.alt,self.data["home_lat"],self.data["home_lon"],self.data["home_alt"])<1:
            self.data["payload"] = self.data["batch_size"]
            if self.calledby == 'CheckPayLoad':
                # self.state["ready_for_next_batch"] = True
                self.state["is_home"] = True
                self.data["payload"] = self.data["batch_size"]
                print("Restocking the payload")
                self.state["is_armed"] = False
                # # self.vehicle.mode = VehicleMode("GUIDED")
                self.vehicle.armed = False
                time.sleep(15)
                print("Restocking the payload")
            self.update_response(self.data)
            return Status.SUCCESS

        return Status.RUNNING
    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")



    def get_dist(self, lat1, lon1, alt1, lat2, lon2, alt2):
        R = 6371000  # Earth radius in meters

        # Convert degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Differences
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Haversine (horizontal distance)
        a = math.sin(dlat / 2)**2 + \
            math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.asin(math.sqrt(a))

        horizontal_dist = R * c  # meters

        # Vertical distance
        vertical_dist = alt2 - alt1  # meters

        # 3D distance
        distance = math.sqrt(horizontal_dist**2 + vertical_dist**2)

        return distance



    def update_response(self,data):
         with open(self.state["json_address"], "w") as f:
        # 2. ADDED indent=4 for proper JSON formatting
            json.dump(data, f, indent=4)
