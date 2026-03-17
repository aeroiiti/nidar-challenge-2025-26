#!/usr/bin/env python3
"""
BACKUP CODE: Simple GPS Logging When Person Detected in Bottom 1/8th of Frame
Uses drone's GPS from shared file (no direct drone connection)
With 1-meter minimum distance filter between detections
"""

import cv2
import numpy as np
from ultralytics import YOLO
from collections import defaultdict
import torch
import threading
import queue
import time
import csv
import subprocess
import json
import sys
import os
from datetime import datetime
from math import radians, cos, sin, asin, sqrt

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
model_path = BASE_DIR / "models" / "yolov8s_visdrone.pt"

# ==========================
# CONFIGURATION
# ==========================
# Shared GPS file (written by main scouting script)
GPS_SHARE_FILE = "/tmp/drone_gps.json"

# Camera settings
CAMERA_INDEX = 0  # USB webcam (change to 1, 2, etc. if you have multiple cameras)
INFER_W, INFER_H = 640, 384

# Detection parameters
CONF_THRESHOLD = 0.35
ASPECT_RATIO_MIN = 0.1
ASPECT_RATIO_MAX = 16
MIN_HEIGHT = 15

# GPU Configuration for Jetson
DEVICE = "0"  # Use GPU

# Output JSON file
OUTPUT_JSON = "detected_persons.json"

# Detection zone: bottom 1/8th of the frame
BOTTOM_ZONE_FRACTION = 2.0 / 8.0
LEFT_EXCLUSION_FRACTION = 1.0 / 10.0
RIGHT_EXCLUSION_FRACTION = 1.0 / 10.0

# Minimum distance between detections (meters)
MIN_DETECTION_DISTANCE = 1.0

# ==========================
# GLOBAL STATE
# ==========================
class SharedState:
    def __init__(self):
        self.running = True
        self.frame_queue = queue.Queue(maxsize=2)
        self.result_queue = queue.Queue(maxsize=2)
        self.reported_tracks = set()  # Track IDs we've already saved
        self.lock = threading.Lock()
        self.person_counter = 0  # Sequential person ID
        self.csv_file = None
        self.csv_writer = None
        self.detections = []
        self.serial_process = None

shared = SharedState()


# ==========================
# GPS READING FROM SHARED FILE
# ==========================
def read_drone_gps():
    """
    Read current drone GPS from shared file
    Returns GPS data dict or None if unavailable
    """
    try:
        if not os.path.exists(GPS_SHARE_FILE):
            return None
            
        with open(GPS_SHARE_FILE, 'r') as f:
            gps_data = json.load(f)
        
        # Check if data is recent (within 3 seconds)
        current_time = time.time()
        gps_timestamp = gps_data.get('timestamp', 0)
        
        if current_time - gps_timestamp > 3:
            # Data is stale
            return None
        
        return gps_data
        
    except FileNotFoundError:
        return None
    except json.JSONDecodeError:
        return None
    except Exception as e:
        return None


# ==========================
# DISTANCE CALCULATION
# ==========================
def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    Returns distance in meters
    """
    # Convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    
    # Radius of earth in meters
    r = 6371000
    return c * r


def is_too_close_to_existing(new_lat, new_lon, min_distance=MIN_DETECTION_DISTANCE):
    """
    Check if new coordinates are within min_distance meters of any existing detection
    Returns True if too close (should skip), False if far enough (should log)
    """
    for detection in shared.detections:
        existing_lat = detection['lat']
        existing_lon = detection['lon']
        distance = haversine_distance(new_lat, new_lon, existing_lat, existing_lon)
        if distance < min_distance:
            return True, distance
    return False, None


# ==========================
# SERIAL COMMUNICATION LAUNCHER
# ==========================
def start_serial_communication():
    """Launch the serial communication script in a new terminal window"""
    if shared.serial_process is not None:
        print("[WARN] Serial communication already running")
        return

    serial_script_path = BASE_DIR / "scout_send_serial.py"
    
    if not os.path.exists(serial_script_path):
        print(f"[ERROR] Serial script not found: {serial_script_path}")
        return
    
    print(f"[INFO] Launching serial communication in new terminal: {serial_script_path}")
    
    try:
        # Try gnome-terminal first
        shared.serial_process = subprocess.Popen([
            'gnome-terminal',
            '--',
            'bash', '-c',
            f'taskset -c 3,4,5 {sys.executable} {serial_script_path}; exec bash'
        ])
        print(f"[INFO] ✓ Serial communication started in new terminal (PID: {shared.serial_process.pid})")
    except Exception as e:
        print(f"[WARN] gnome-terminal failed: {e}, trying xterm...")
        try:
            # Fallback to xterm
            shared.serial_process = subprocess.Popen([
                'xterm',
                '-e',
                f'taskset -c 3,4,5 {sys.executable} {serial_script_path}'
            ])
            print(f"[INFO] ✓ Serial communication started in new terminal with xterm (PID: {shared.serial_process.pid})")
        except Exception as e2:
            print(f"[ERROR] Failed to start serial communication in new terminal: {e2}")
            shared.serial_process = None


def save_json():
    """Save detections to JSON file"""
    try:
        data = {"geotag": shared.detections}
        with open(OUTPUT_JSON, 'w') as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        print(f"[ERROR] Failed to save JSON: {e}")


def stop_serial_communication():
    """Stop the serial communication process"""
    if shared.serial_process is None:
        return
    
    print("[INFO] Stopping serial communication...")
    try:
        shared.serial_process.terminate()
        shared.serial_process.wait(timeout=5)
        print("[INFO] ✓ Serial communication stopped")
    except subprocess.TimeoutExpired:
        print("[WARN] Serial process didn't terminate, killing...")
        shared.serial_process.kill()
        shared.serial_process.wait()
    except Exception as e:
        print(f"[ERROR] Error stopping serial process: {e}")
    finally:
        shared.serial_process = None


# ==========================
# DETECTOR CLASS
# ==========================
class SimplePersonDetector:
    def __init__(self, model_path, conf_threshold, aspect_ratio_min, 
                 aspect_ratio_max, min_height):
        print(f"Loading YOLO model on GPU: {model_path}")
        self.yolo = YOLO(model_path)
        
        self.conf_threshold = conf_threshold
        self.aspect_ratio_min = aspect_ratio_min
        self.aspect_ratio_max = aspect_ratio_max
        self.min_height = min_height
        
        self.track_age = defaultdict(int)
        self.locked_tracks = {}
        
        fps = 24
        self.LIKELY_SURVIVOR_FRAMES = fps
        self.SURVIVOR_FRAMES = int(1.5 * fps)

    def filter_by_geometry(self, bbox):
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1
        
        if h < self.min_height or w < 5:
            return False
        
        ar = h / w if w > 0 else 0
        if ar < self.aspect_ratio_min or ar > self.aspect_ratio_max:
            return False
        
        return True

    def adaptive_confidence(self, bbox, base_conf):
        h = bbox[3] - bbox[1]
        if h < 40:
            boost = 0.15
        elif h < 80:
            boost = 0.10
        else:
            boost = 0.0
        return min(1.0, base_conf + boost)
    
    def update_survivor_status(self, track_id):
        age = self.track_age[track_id]
        status = None
        locked = False

        if age >= self.SURVIVOR_FRAMES:
            status = "survivor"
            locked = True
        elif age >= self.LIKELY_SURVIVOR_FRAMES:
            status = "likely_survivor"
            locked = True

        if locked:
            self.locked_tracks[track_id] = status
        elif track_id in self.locked_tracks:
            status = self.locked_tracks[track_id]
            locked = True

        return locked, status

    def detect_frame(self, frame_resized):
        results = self.yolo.track(
            frame_resized,
            conf=self.conf_threshold,
            classes=[0],
            tracker="botsort.yaml",
            persist=True,
            device=DEVICE,
            half=True,  # Enable FP16 for Jetson
            imgsz=INFER_W,
            verbose=False
        )[0]
        
        detections = []
        if results.boxes is None or results.boxes.id is None:
            return detections
        
        for box, conf, tid in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.id):
            bbox = list(map(int, box.tolist()))
            
            if not self.filter_by_geometry(bbox):
                continue
            
            conf = self.adaptive_confidence(bbox, float(conf))
            tid = int(tid)
            
            self.track_age[tid] += 1
            locked, status = self.update_survivor_status(tid)
            is_stable = self.track_age[tid] >= 3
            
            detections.append((bbox, conf, tid, is_stable, locked, status))
        
        return detections


# ==========================
# DETECTION ZONE CHECK
# ==========================
def is_in_bottom_zone(bbox, frame_height):
    """
    Check if bbox bottom edge is in the bottom 1/8th of the frame
    """
    x1, y1, x2, y2 = bbox
    bottom_threshold = frame_height * (1.0 - BOTTOM_ZONE_FRACTION)
    
    # Check if the bottom of the bbox is in the detection zone
    return y2 >= bottom_threshold


def is_in_horizontal_zone(bbox, frame_width):
    """
    Check if bbox is in the middle 8/10th horizontally (exclude 1/10th from each side)
    """
    x1, y1, x2, y2 = bbox
    left_threshold = frame_width * LEFT_EXCLUSION_FRACTION
    right_threshold = frame_width * (1.0 - RIGHT_EXCLUSION_FRACTION)
    
    # Check if bbox center is within the allowed horizontal zone
    cx = (x1 + x2) / 2
    return left_threshold <= cx <= right_threshold


def is_centroid_in_bottom_zone(bbox, frame_height):
    """
    Check if bbox centroid is in the bottom 1/8th of the frame
    """
    x1, y1, x2, y2 = bbox
    bottom_threshold = frame_height * (1.0 - BOTTOM_ZONE_FRACTION)
    
    # Check centroid position
    cy = (y1 + y2) / 2
    return cy >= bottom_threshold


# ==========================
# THREAD 1: INPUT (Camera Capture)
# ==========================
def input_thread(camera_source):
    print(f"Starting camera capture from: {camera_source}")

    def open_camera():
        cap = cv2.VideoCapture(camera_source)
        if cap.isOpened():
            print(f"✓ Camera connected: "
                  f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                  f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
            return cap
        return None

    cap = None
    while cap is None and shared.running:
        cap = open_camera()
        if cap is None:
            print("[WARN] USB camera not ready, retrying...")
            time.sleep(1)

    fail_count = 0
    MAX_FAIL = 15  # ~0.5s at 30 FPS

    while shared.running:
        ret, frame = cap.read()

        if not ret:
            fail_count += 1
            print(f"[WARN] USB camera frame drop ({fail_count})")

            if fail_count >= MAX_FAIL:
                print("[WARN] USB camera stalled, reconnecting...")
                cap.release()
                cap = None
                time.sleep(0.5)

                while cap is None and shared.running:
                    cap = open_camera()
                    if cap is None:
                        time.sleep(1)

                fail_count = 0

            continue

        fail_count = 0

        try:
            shared.frame_queue.put(frame, timeout=0.05)
        except queue.Full:
            pass

    if cap:
        cap.release()
    print("Input thread stopped")


# ==========================
# THREAD 2: PROCESSING (Detection + GPS from shared file)
# ==========================
def processing_thread(detector):
    print("Starting processing thread")
    
    gps_warning_shown = False  # Track if we've shown GPS warning
    
    while shared.running:
        try:
            frame = shared.frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        
        H, W = frame.shape[:2]
        resized = cv2.resize(frame, (INFER_W, INFER_H))
        
        detections = detector.detect_frame(resized)
        
        scale_x = W / INFER_W
        scale_y = H / INFER_H
        
        annotated_frame = frame.copy()
        
        # Draw detection zone line
        zone_y = int(H * (1.0 - BOTTOM_ZONE_FRACTION))
        cv2.line(annotated_frame, (0, zone_y), (W, zone_y), (255, 0, 255), 2)
        cv2.putText(annotated_frame, "DETECTION ZONE", (10, zone_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        
        # Draw vertical exclusion zones
        left_x = int(W * LEFT_EXCLUSION_FRACTION)
        right_x = int(W * (1.0 - RIGHT_EXCLUSION_FRACTION))
        cv2.line(annotated_frame, (left_x, 0), (left_x, H), (255, 165, 0), 2)
        cv2.line(annotated_frame, (right_x, 0), (right_x, H), (255, 165, 0), 2)
        cv2.putText(annotated_frame, "ACTIVE ZONE", (left_x + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
        
        for bbox, conf, tid, is_stable, locked, status in detections:
            x1, y1, x2, y2 = bbox
            x1 = int(x1 * scale_x)
            y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x)
            y2 = int(y2 * scale_y)
            
            scaled_bbox = (x1, y1, x2, y2)
            in_horizontal_zone = is_in_horizontal_zone(scaled_bbox, W)
    
            # Skip detections outside the horizontal zone
            if not in_horizontal_zone:
                continue
            
            # Check if bottom is in detection zone
            in_zone = is_in_bottom_zone(scaled_bbox, H)
            
            # Check if centroid is in bottom zone (for GPS logging)
            centroid_in_bottom = is_centroid_in_bottom_zone(scaled_bbox, H)
            
            # Log GPS if person is in bottom zone and not already reported
            if is_stable and centroid_in_bottom and tid not in shared.reported_tracks:
                # 🔧 READ GPS FROM SHARED FILE (no direct drone connection)
                gps_data = read_drone_gps()
                
                if gps_data:
                    # GPS data is available
                    if not gps_warning_shown:
                        print("[INFO] ✓ GPS data available from main script")
                        gps_warning_shown = True
                    
                    try:
                        lat = gps_data['lat']
                        lon = gps_data['lon']
                        alt = gps_data.get('alt', 0)
                        
                        # Check if this location is too close to existing detections
                        too_close, distance = is_too_close_to_existing(lat, lon)
                        
                        if not too_close:
                            with shared.lock:
                                if tid not in shared.reported_tracks:
                                    shared.person_counter += 1
                                    person_id = shared.person_counter
                                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                                    
                                    detection_entry = {
                                        "person_id": person_id,
                                        "track_id": tid,
                                        "lat": round(lat, 7),
                                        "lon": round(lon, 7),
                                        "received": False
                                    }
                                    
                                    # Print to console
                                    print(f"\n{'='*60}")
                                    print(f"✓ PERSON LOGGED - Person ID: {person_id}, Track ID: {tid}")
                                    print(f"  Drone GPS: {lat:.7f}, {lon:.7f}")
                                    print(f"  Drone Altitude: {alt:.2f}m")
                                    print(f"  Timestamp: {timestamp}")
                                    print(f"{'='*60}\n")
                                    
                                    # Save to JSON
                                    shared.detections.append(detection_entry)
                                    save_json()
                                
                                    shared.reported_tracks.add(tid)
                        else:
                            # Skip logging but still mark as reported to avoid repeated checks
                            with shared.lock:
                                if tid not in shared.reported_tracks:
                                    shared.reported_tracks.add(tid)
                                    print(f"[INFO] ✗ Skipped Track ID {tid} - within {distance:.2f}m of existing detection (min: {MIN_DETECTION_DISTANCE}m)")
                                
                    except Exception as e:
                        print(f"[ERROR] Error processing GPS data: {e}")
                else:
                    # GPS data not available - skip logging silently (only show warning once)
                    if not gps_warning_shown:
                        print("[INFO] ⚠ GPS data not available - detections will not be logged")
                        print("[INFO]   (Waiting for main script to provide GPS data)")
                        gps_warning_shown = True
            
            # Draw bounding box (different colors for locked + in zone)
            if locked and in_zone:
                color = (0, 0, 255)  # Red if LOCKED and in zone
                thickness = 3
            elif locked:
                color = (255, 0, 255)  # Magenta if LOCKED but not in zone
                thickness = 2
            elif in_zone:
                color = (0, 165, 255)  # Orange if in zone but not locked
                thickness = 2
            else:
                color = (0, 255, 0) if is_stable else (0, 255, 255)
                thickness = 2
            
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, thickness)
            
            # Draw label
            zone_status = "IN ZONE" if in_zone else ""
            label = f"ID:{tid} {conf:.2f} {zone_status}"
            cv2.putText(annotated_frame, label, (x1, max(0, y1 - 10)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw center point
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(annotated_frame, (cx, cy), 5, (255, 0, 0), -1)
        
        # Add info overlay
        gps_status = "GPS: ✓" if read_drone_gps() else "GPS: ✗"
        info_text = f"Detections: {len(detections)} | Logged: {len(shared.detections)} | {gps_status}"
        cv2.putText(annotated_frame, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        try:
            shared.result_queue.put(annotated_frame, timeout=0.1)
        except queue.Full:
            pass
    
    print("Processing thread stopped")


# ==========================
# THREAD 3: OUTPUT (Display)
# ==========================
def output_thread():
    print("Starting output thread")
    
    while shared.running:
        try:
            frame = shared.result_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        
        cv2.imshow("Simple Person Detection - Bottom Zone", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # q or ESC
            shared.running = False
            break
    
    cv2.destroyAllWindows()
    print("Output thread stopped")


# ==========================
# MAIN
# ==========================
def main():
    print("="*60)
    print("Person Detection with Shared GPS (No Direct Drone Connection)")
    print("Reads GPS from file written by main scouting script")
    print(f"Minimum detection distance: {MIN_DETECTION_DISTANCE}m")
    print("="*60)
    print(f"PyTorch version: {torch.__version__}")
    print(f"CUDA available: {torch.cuda.is_available()}")
    print(f"CUDA device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A'}")
    print("="*60)
    
    # Initialize JSON file with empty array
    shared.detections = []
    save_json()
    print(f"✓ JSON file initialized: {OUTPUT_JSON}")

    # Launch serial communication in new terminal
    start_serial_communication()
    time.sleep(1)  # Give serial process time to initialize
    
    # 🔧 NO DRONE CONNECTION - Read from shared file instead
    print(f"✓ Will read GPS from shared file: {GPS_SHARE_FILE}")
    print("  (Main scouting script must be running and providing GPS data)")
    print("="*60)
    
    # Initialize detector
    detector = SimplePersonDetector(
        model_path=BASE_DIR / "models" / "yolov8s_visdrone.pt",
        conf_threshold=CONF_THRESHOLD,
        aspect_ratio_min=ASPECT_RATIO_MIN,
        aspect_ratio_max=ASPECT_RATIO_MAX,
        min_height=MIN_HEIGHT
    )
    
    # Start threads (note: processing_thread no longer needs vehicle parameter)
    print("\nStarting threads...")
    t1 = threading.Thread(target=input_thread, args=(CAMERA_INDEX,), daemon=True)
    t2 = threading.Thread(target=processing_thread, args=(detector,), daemon=True)
    t3 = threading.Thread(target=output_thread, daemon=True)
    
    t1.start()
    t2.start()
    t3.start()
    
    print("✓ All threads started")
    print(f"\nDetection zone: Bottom {BOTTOM_ZONE_FRACTION*100:.0f}% of frame")
    print("Press 'q' or ESC in the video window to quit\n")
    
    # Wait for threads
    try:
        while shared.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        shared.running = False
    
    # Cleanup
    print("\nShutting down...")
    t1.join(timeout=2)
    t2.join(timeout=2)
    t3.join(timeout=2)
    stop_serial_communication()

    # Close JSON file
    print(f"✓ JSON file saved: {OUTPUT_JSON}")
    print(f"✓ Total persons detected: {len(shared.detections)}")
    print("✓ Shutdown complete")


if __name__ == "__main__":
    main()