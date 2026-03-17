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


from behaviours.path_planner import PathPlanner
class CheckBatchReady(Behaviour):
    def __init__(self, name,state,calledby=""):
        super(CheckBatchReady,self).__init__(name)
        self.state = state
        self.calledby = calledby
        self.data = None
    def update(self):
        with open(self.state["json_address"],"r") as knw:
            self.data = json.load(knw)

        if self.calledby == "start" and not self.state["is_home"]:
            return Status.SUCCESS
    
        available_geotags = len(self.data["geo_taged_positions"])
        visited_tags = self.data["indexed_reached"]

        new_geotags = available_geotags-visited_tags
        if self.data["batch_planned"]==False:
            if new_geotags<self.data["batch_size"]:
                return Status.RUNNING
            elif new_geotags>=self.data["batch_size"]:
                self.data["batch_planned"] = True
                self.data["best_plan"] = (PathPlanner.find_shortest_path(self.data["geo_taged_positions"][self.data["indexed_reached"]:self.data["indexed_reached"]+self.data["batch_size"]],self.data["home_lat"],self.data["home_lon"]))
                self.update_response(self.data)
                return Status.SUCCESS
            
        else:
            return Status.SUCCESS
    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

    def update_response(self,data):
        with open(self.state["json_address"],"w") as f:
            json.dump(data,f,indent=4)
