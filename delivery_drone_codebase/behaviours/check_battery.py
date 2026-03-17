#!/usr/bin/env python3
import time 
import math
import re
import py_trees
from py_trees.common import Status
from py_trees.decorators import Inverter
from py_trees import logging as log_tree
from py_trees.behaviour import Behaviour   
from py_trees.composites import Selector,Sequence,Parallel
import numpy as np
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative




class CheckBattery(Behaviour):
    def __init__(self,name,vehicle,threshold):
        super(CheckBattery,self).__init__(name=name)
        # self.expected_value = expected_value
        # self.blackboard_variable = blackboard_variable
        self.threshold = threshold
        self.vehicle = vehicle
        self.match = re.search(r"level=(\d+)", str(vehicle.battery))
        self.level = 0
        if self.match:
            self.level = int(self.match.group(1))
            print("Battery Level:", self.level)
        self.battery = self.level
    def update(self):
        self.match = re.search(r"level=(\d+)", str(self.vehicle.battery))
        if self.match:
            self.level = int(self.match.group(1))
            # print("Battery Level:", level)
       
             
        self.battery = self.level
        if self.battery <self.threshold:
            print('Returning to Launch positon')
            return Status.FAILURE
        print(self.battery)
        return Status.SUCCESS
    def terminate(self, new_status):

        self.logger.debug(f"Action::terminate {self.name} to {new_status}")