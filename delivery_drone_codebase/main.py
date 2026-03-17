#!/usr/bin/env python3

import time
from py_trees.composites import Sequence, Parallel,Selector
from py_trees.common import ParallelPolicy

from py_trees import logging as log_tree
from dronekit import connect
import json
from behaviours.arm_action import ArmAction
from behaviours.check_battery import CheckBattery
from behaviours.check_payload import CheckPayload
from behaviours.rtl_action import RTLAction
from behaviours.check_batch_ready import CheckBatchReady
# from behaviours.geo_tag_read_sim import ReceiveLocationsSimulator
from behaviours.geotag_read import ReceiveLocations
from behaviours.goto_waypoint import GoToWayPoint
from behaviours.drone_allign import AlignAction
import threading

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

drone_connect = "/dev/ttyACM0"
#drone_connect = "tcp:127.0.0.1:5762"

telemetry_connect = "/dev/ttyUSB0"
# Shared states
shared_states = {
    
    "json_address": str(BASE_DIR / "data" / "received_geotags.json"),
    "is_home":True,
    "is_armed":False
}
def create_root(vehicle):
    root = Sequence("Root",memory=True)

    arm = ArmAction("Arm",vehicle,shared_states)
    priority = Sequence("Priority",memory=False)
    check_battery_node = Selector('Check Battery',memory=False)
    cb = CheckBattery("cbn", vehicle,0)
    rtl_cb = RTLAction("RTL_CB","CheckBattery",vehicle,shared_states)
    check_battery_node.add_children([cb,rtl_cb])

    check_payload_node = Selector("cbp",memory=False)
    cp = CheckPayload("CheckPayload",shared_states)
    rtl_cp = RTLAction("RTL_CP","CheckPayLoad",vehicle,shared_states)
    check_payload_node.add_children([cp,rtl_cp])
    # check_payload_node.add_child(cp)

    MissionExecutionNode = Sequence(
    "Mission_node",memory=True
    )

    check_batch_start = CheckBatchReady("CheckBatchReadyStart",shared_states,"start")
    

    check_batch = CheckBatchReady("CheckBatchReady",shared_states)

    gotowaypoint = GoToWayPoint("gtwp",vehicle,shared_states)

    drone_allign_action = AlignAction("DroneAlignAction",vehicle,shared_states)

    MissionExecutionNode.add_children([check_batch,gotowaypoint,drone_allign_action])

    priority.add_children([check_battery_node,check_payload_node,MissionExecutionNode])

    root.add_children([check_batch_start,arm,priority])
    return root 
def update_response(data):
    with open(shared_states["json_address"],"w") as f:
        json.dump(data,f,indent=4)
if __name__ == "__main__":
    data = None
    log_tree.level = log_tree.Level.DEBUG

    # vehicle = connect("127.0.0.1:14550", wait_ready=True)
    vehicle = connect(drone_connect, wait_ready=True)
    
    with open(shared_states["json_address"],"r") as knw:
        data = json.load(knw)
    data["home_lat"] = vehicle.location.global_frame.lat
    data["home_lon"] = vehicle.location.global_frame.lon
    data["home_alt"] = vehicle.location.global_frame.alt

    update_response(data)
    tree = create_root(vehicle)
    receiveLoc = ReceiveLocations(shared_states)
    th = threading.Thread(target=receiveLoc.read)
    th.daemon = True
    th.start()
    
#    tree.setup(timeout=15)
    try:
        while True:
            tree.tick_once()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
