#!/usr/bin/env python3
import time 
import math
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
class ArmAction(Behaviour):
    def __init__(self,name,vehicle,shared_states):
        super(ArmAction,self).__init__(name=name)
        self.DEFAULT_TAKEOFF_THRUST = 1
        self.SMOOTH_TAKEOFF_THRUST = 1
        self.vehicle = vehicle
        self.states = shared_states
        self.data = None
    def setup(self):
        self.logger.debug(f"Action::setup {self.name}")
    def initialise(self):
        print('Basic pre-arm Checks')
        while not self.vehicle.is_armable:
            print('Waiting for vehicle to initialise')
            time.sleep(1)
        print('Arming motors')
        # vehicle.mode = VehicleMode('STABILIZE')

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print('Waiting for arming...')
            # self.vehicle.armed = True
            time.sleep(1)
        
        print('MOTORS ARE ARMED!!!')
        print ("Taking off!")

        # self.set_altitude_target(10)

        self.logger.debug(f"Action::Initialise {self.name}")
    def update(self):
        with open(self.states["json_address"],"r") as knw:
            self.data = json.load(knw)
        
        if self.states["is_armed"]==True:
            return Status.SUCCESS
        # print('Basic pre-arm Checks')
        # while not self.vehicle.is_armable:
        #     print('Waiting for vehicle to initialise')
        #     time.sleep(1)
        # print('Arming motors')
        # # vehicle.mode = VehicleMode('STABILIZE')

        # self.vehicle.mode = VehicleMode("GUIDED")
        # self.vehicle.armed = True

        # while not self.vehicle.armed:
        #     print('Waiting for arming...')
        #     self.vehicle.armed = True
        #     time.sleep(1)
        
        print('MOTORS ARE ARMED!!!')
        print ("Taking off!")
        # thrust = self.DEFAULT_TAKEOFF_THRUST
        current_altitude = self.vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, 8))
        if current_altitude >= 8*0.90: # Trigger just below target alt.
            print("Reached target altitude")
            self.states["is_armed"] = True
            self.states["is_home"] = False
            self.update_response(self.data)
            return Status.SUCCESS
            
        self.set_altitude_target(8)
        time.sleep(0.2)
        return Status.RUNNING
    def terminate(self, new_status):

        self.logger.debug(f"Action::terminate {self.name} to {new_status}")
    def set_altitude_target(self,target_altitude):
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,       # confirmation
            0, 0, 0, 0, 0, 0,
            target_altitude)      # 10 meters altitude
        self.vehicle.send_mavlink(msg)
    def update_response(self,data):
        with open(self.states["json_address"],"w") as f:
            json.dump(data,f,indent=4)
