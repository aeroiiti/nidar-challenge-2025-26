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
from behaviours.deploy_action import Action
class GoToWayPoint(Behaviour):
    def __init__(self, name,vehicle,state):
        super(GoToWayPoint,self).__init__(name)
        self.currentPosid = 0
        self.vehicle = vehicle
        self.state = state

        self.data = None
        self.lat = None
        self.long = None
    def initialise(self):
        with open(self.state["json_address"],"r") as knw:
            self.data = json.load(knw)

        self.lat = self.data["best_plan"][self.data["currentPosid"]][1]
        self.long = self.data["best_plan"][self.data["currentPosid"]][2]

        
        print('lat',self.lat,'long: ',self.long)
        # self.goto_position_target_global_int(self.state["geo_taged_positions"][self.state["indexed_reached"]][1],self.state["geo_taged_positions"][self.state["indexed_reached"]][2])
        
        
        location = LocationGlobalRelative(self.lat,self.long,8)
        self.vehicle.simple_goto(location, airspeed=0.6)
        print('other half reached')


        self.logger.debug(f"Action::Initialise {self.name}")

    def update(self):
        
        print('GoToActionIsRunning')
        print(f'NORTH: {self.vehicle.location.local_frame.north} , EAST: {self.vehicle.location.local_frame.east} , DOWN: {self.vehicle.location.local_frame.down} ')
        print()
        print()
        print("current_pos_id",self.data["currentPosid"])
        print()
        print()
        print('Going to the location: ',self.data["best_plan"][self.data["currentPosid"]])
        # self.vehicle.mode = VehicleMode("GUIDED")

        
        
        
        
        if self.get_dist(self.lat,self.long,self.vehicle.location.global_frame.lat,self.vehicle.location.global_frame.lon) <0.9:
            # Positions[3] = 1
            with open(self.state["json_address"],"r") as knw:
                self.data = json.load(knw)
            print('WE reached at : ',self.data["best_plan"][self.data["currentPosid"]])
            print("--Alligning the drone...")
            self.data["currentPosid"] += 1
            # self.data["payload"] -= 1
            if self.data["payload"] <= 0:
                  self.data["currentPosid"] = 0
                  self.data["batch_planned"] = False
            self.data["indexed_reached"] += 1
            self.update_response(self.data)
            # Action(self.vehicle)
            return Status.SUCCESS

        return Status.RUNNING
    
    def terminate(self, new_status):

        self.logger.debug(f"Action::terminate {self.name} to {new_status}")
        
    def get_dist(self,lat1, lon1, lat2, lon2):
        R = 6371  # Earth radius in km
        
        # Convert degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Differences
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return R * c * 1000
    def update_response(self,data):
         with open(self.state["json_address"], "w") as f:
        # 2. ADDED indent=4 for proper JSON formatting
            json.dump(data, f, indent=4)

    
