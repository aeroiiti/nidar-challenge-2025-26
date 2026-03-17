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


class CheckPayload(Behaviour):
    def __init__(self, name,state):
        super(CheckPayload,self).__init__(name)
        self.state = state
        self.data = None
    def update(self):
        try:
            with open(self.state["json_address"],"r") as knw:
                self.data = json.load(knw)

        except FileNotFoundError:
            print(f"not found")
        if self.data["payload"] <1:
            return Status.FAILURE
        return Status.SUCCESS
    def terminate(self, new_status):

        self.logger.debug(f"Action::terminate {self.name} to {new_status}")
