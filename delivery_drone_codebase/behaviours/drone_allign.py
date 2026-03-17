#!/usr/bin/env python3

import time
import os
import threading
import json
import cv2
from pathlib import Path

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from deploy_action import Action

from pymavlink import mavutil

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import hailo
from hailo_apps.hailo_app_python.core.common.buffer_utils import (
    get_caps_from_pad,
    get_numpy_from_buffer,
)
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import (
    GStreamerDetectionApp,
)
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import (
    app_callback_class,
)
from hailo_apps.hailo_app_python.core.common.core import get_default_parser

from behaviours.deploy_action import Action


from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
hef_path = BASE_DIR / "models" / "yolov8s_new.hef"
# =============================================================================
# MAVLINK BODY NED VELOCITY
# =============================================================================
def send_body_ned_velocity(vehicle, fwd, right, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        fwd, right, down,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)


# =============================================================================
# HAILO CALLBACK DATA HOLDER
# =============================================================================
class HailoDetectionCallback(app_callback_class):
    def __init__(self):
        super().__init__()
        self.latest_detections = []
        self.latest_frame = None
        self.w = 640
        self.h = 480
        self.lock = threading.Lock()
        self.last_detection_time = time.time()

    def get(self):
        with self.lock:
            return (
                self.latest_detections.copy(),
                self.latest_frame,
                self.w,
                self.h,
            )

    def set_size(self, w, h):
        with self.lock:
            self.w = w
            self.h = h


# =============================================================================
# HAILO PAD CALLBACK
# =============================================================================
def hailo_callback(pad, info, user_data):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    fmt, w, h = get_caps_from_pad(pad)
    if w and h:
        user_data.set_size(w, h)

    frame = None
    if user_data.use_frame and fmt:
        frame = get_numpy_from_buffer(buffer, fmt, w, h)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

    persons = []
    for d in detections:
        if d.get_label() != "person":
            continue
        b = d.get_bbox()
        persons.append({
            "confidence": d.get_confidence(),
            "bbox": (
                int(b.xmin() * w),
                int(b.ymin() * h),
                int(b.xmax() * w),
                int(b.ymax() * h),
            ),
        })

    with user_data.lock:
        user_data.latest_detections = persons
        user_data.latest_frame = frame
        if persons:
            user_data.last_detection_time = time.time()

    return Gst.PadProbeReturn.OK


# =============================================================================
# ALIGN ACTION (BT LEAF NODE)
# =============================================================================
class AlignAction(Behaviour):
    def __init__(self, name, vehicle, shared_states):
        super().__init__(name)
        self.vehicle = vehicle
        self.states = shared_states

        # --- tuning ---
        self.pixel_threshold = 80
        self.vel_xy = 0.4
        self.no_detection_timeout = 15.0

        self.start_time = None
        self.hailo_data = None
        self.hailo_app = None

        self.data = None
    # -------------------------------------------------------------------------
    def setup(self, **kwargs):
        self.hailo_data = HailoDetectionCallback()
        parser = get_default_parser()
        parser.set_defaults(input="rpi", hef_path=str(hef_path))

        self.hailo_app = GStreamerDetectionApp(
            hailo_callback,
            self.hailo_data,
            parser=parser
        )

        threading.Thread(target=self.hailo_app.run, daemon=True).start()
        time.sleep(3)

    # -------------------------------------------------------------------------
    def initialise(self):
        #self.setup()
        self.hailo_data = HailoDetectionCallback()
        parser = get_default_parser()
        parser.set_defaults(input="rpi", hef_path=str(hef_path))

        self.hailo_app = GStreamerDetectionApp(
            hailo_callback,
            self.hailo_data,
            parser=parser
        )

        threading.Thread(target=self.hailo_app.run, daemon=True).start()
        time.sleep(3)

        
        self.start_time = time.time()

    # -------------------------------------------------------------------------
    def update(self):
        #self.setup()
        detections, frame, w, h = self.hailo_data.get()

        # ================= NO DETECTION TIMEOUT =================
        if not detections:
            elapsed = time.time() - self.hailo_data.last_detection_time
            if elapsed > self.no_detection_timeout:
                print("[ALIGN] No detection for 15s → moving to next position")
                send_body_ned_velocity(self.vehicle, 0, 0, 0)
                self._advance_index("not_detected")
                return Status.SUCCESS
            return Status.RUNNING

        # ================= ALIGNMENT =================
        best = max(detections, key=lambda d: d["confidence"])
        x1, y1, x2, y2 = best["bbox"]

        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        err_x = cx - (w / 2)
        err_y = cy - (h / 2)

        vel_fwd = 0.0
        vel_right = 0.0
        vel_down = 0.0

        if abs(err_x) > self.pixel_threshold:
            vel_right = self.vel_xy if err_x > 0 else -self.vel_xy

        if abs(err_y) > self.pixel_threshold:
            vel_fwd = -self.vel_xy if err_y > 0 else self.vel_xy

        send_body_ned_velocity(self.vehicle, vel_fwd, vel_right, vel_down)

        print(
            f"[ALIGN] FWD:{vel_fwd:+.2f} "
            f"RIGHT:{vel_right:+.2f} "
            f"DOWN:{vel_down:+.2f}"
        )

        if frame is not None:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imshow("Drone Align", frame)
            cv2.waitKey(1)

        # ================= ALIGNED =================
        if abs(err_x) <= self.pixel_threshold and abs(err_y) <= self.pixel_threshold:
            print("[ALIGN] Target aligned → moving to next position")
            send_body_ned_velocity(self.vehicle, 0, 0, 0)
            self._advance_index("alligned")
            
            print("Dropping payload...")
            Action(self.vehicle)
            return Status.SUCCESS

        return Status.RUNNING

    # -------------------------------------------------------------------------
    def terminate(self, new_status):
        send_body_ned_velocity(self.vehicle, 0, 0, 0)

    # -------------------------------------------------------------------------
    def _advance_index(self,calledby):
        if(calledby!="alligned"):
            with open(self.states["json_address"], "r") as f:
                self.data = json.load(f)

            self.data["currentPosid"] += 1
            self.data["indexed_reached"] +=1
            with open(self.states["json_address"], "w") as f:
                json.dump(self.data, f, indent=4)
        else:
            with open(self.states["json_address"], "r") as f:
                self.data = json.load(f)

            self.data["currentPosid"] += 1
            self.data["payload"] -= 1
            if self.data["payload"]<=0:
                self.data["currentPosid"] = 0
                self.data["batch_planned"] = False

            self.data["indexed_reached"] +=1
            with open(self.states["json_address"], "w") as f:
                json.dump(self.data, f, indent=4)
