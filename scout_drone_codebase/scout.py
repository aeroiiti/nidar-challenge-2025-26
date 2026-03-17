from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import collections
collections.MutableMapping = collections.abc.MutableMapping
import xml.etree.ElementTree as ET
import re
import matplotlib.path as mplPath
import numpy as np
import sys
import os
import subprocess
import math
import json
import threading
from datetime import datetime

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
kml_file = BASE_DIR / "mission_files" / "cricketground_full.kml"

person_process = None  # global handle
gps_logger_running = False  # Track GPS logger status

# Shared GPS file for camera script
GPS_SHARE_FILE = "/tmp/drone_gps.json"

def gps_logger_thread():
    """
    Continuously write current GPS to shared file for camera script
    Runs while drone is armed or mission is active
    """
    global gps_logger_running
    gps_logger_running = True
    
    print("✓ GPS logger thread started")
    
    while gps_logger_running:
        try:
            gps_data = {
                "lat": vehicle.location.global_frame.lat,
                "lon": vehicle.location.global_frame.lon,
                "alt": vehicle.location.global_relative_frame.alt,
                "timestamp": time.time()
            }
            
            with open(GPS_SHARE_FILE, 'w') as f:
                json.dump(gps_data, f)
                
        except Exception as e:
            print(f"GPS logger error: {e}")
        
        time.sleep(0.5)  # Update GPS twice per second
    
    print("✓ GPS logger thread stopped")


def start_person_detection():
    """
    Start person detection script in a new terminal window
    """
    global person_process
    try:
        # Open geolocation_usb.py in a new terminal
        person_process = subprocess.Popen([
            'gnome-terminal',
            '--',
            'python3',
            str(BASE_DIR / "geolocation.py")
        ])
        print("✓ Started person detection in new terminal")
    except Exception as e:
        print(f"Error starting person detection: {e}")
        # Fallback: try with xterm if gnome-terminal is not available
        try:
            person_process = subprocess.Popen([
                'xterm',
                '-e',
                'python3',
                '/home/raptor/nidar/Jetson_files/geolocation_usb.py'
            ])
            print("✓ Started person detection in new terminal (xterm)")
        except Exception as e2:
            print(f"Error with xterm fallback: {e2}")


# drone_connect = "tcp:127.0.0.1:5762"
# drone_connect = "udp:127.0.0.1:14550"
drone_connect = "/dev/ttyACM0"

# Set parameters
altitude = 6
take_off = False
MISSION_PROGRESS_FILE = "/home/raptor/nidar/Jetson_files/mission_progress.json"

###############################

print("Connecting to vehicle on: %s" % drone_connect)
vehicle = connect(drone_connect, wait_ready=True, baud=57600)

# Download the vehicle waypoints (commands). Wait until download is complete.
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Get the home location
home = vehicle.home_location

vehicle.parameters['RTL_ALT'] = 0
vehicle.parameters['WPNAV_SPEED'] = 200
vehicle.parameters['LAND_SPEED'] = 50


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    global gps_logger_running
    
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # 🔧 START CAMERA BEFORE ARMING
    print("\n" + "="*60)
    print("Starting person detection BEFORE arming...")
    print("(Camera will open but GPS logging will wait for arming)")
    print("="*60 + "\n")
    
    start_person_detection()
    
    # Give camera script time to initialize
    print("Waiting 5 seconds for camera initialization...")
    time.sleep(5)
    print("✓ Camera should be running\n")

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    # 🔧 START GPS LOGGER THREAD after arming
    print("\n" + "="*60)
    print("Vehicle armed! Starting GPS logger...")
    print("="*60 + "\n")
    
    gps_thread = threading.Thread(target=gps_logger_thread, daemon=True)
    gps_thread.start()
    time.sleep(1)  # Give GPS logger time to write first update

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    # (otherwise the command after Vehicle.simple_takeoff will execute
    # immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def save_mission_progress(waypoint_index, total_waypoints, last_completed_wp, status="IN_PROGRESS"):
    """
    Save mission progress to JSON file for resumption capability
    """
    progress_data = {
        "last_completed_index": waypoint_index,
        "total_waypoints": total_waypoints,
        "last_completed_waypoint": last_completed_wp,
        "status": status,
        "timestamp": datetime.now().isoformat(),
        "completion_percentage": round((waypoint_index / total_waypoints) * 100, 2) if total_waypoints > 0 else 0
    }
    
    try:
        with open(MISSION_PROGRESS_FILE, 'w') as f:
            json.dump(progress_data, f, indent=2)
        print(f"✓ Progress saved: {waypoint_index}/{total_waypoints} waypoints completed")
    except Exception as e:
        print(f"Error saving progress: {e}")


def load_mission_progress():
    """
    Load mission progress from JSON file
    Returns the index of the last completed waypoint, or -1 if starting fresh
    """
    if not os.path.exists(MISSION_PROGRESS_FILE):
        print("No previous mission progress found. Starting fresh.")
        return -1
    
    try:
        with open(MISSION_PROGRESS_FILE, 'r') as f:
            progress_data = json.load(f)
        
        if progress_data.get("status") == "COMPLETE":
            print("Previous mission was completed. Starting fresh.")
            return -1
        
        last_index = progress_data.get("last_completed_index", -1)
        percentage = progress_data.get("completion_percentage", 0)
        timestamp = progress_data.get("timestamp", "unknown")
        
        print(f"Found previous mission progress:")
        print(f" - Last completed waypoint: {last_index}")
        print(f" - Completion: {percentage}%")
        print(f" - Last updated: {timestamp}")
        
        return last_index
    
    except Exception as e:
        print(f"Error loading progress file: {e}")
        return -1


def kml_parser(kml_file):
    # KML parse
    k = 0
    tree = ET.parse(kml_file)
    root = tree.getroot()
    points_list = []

    # Identify default namespace
    namespace = re.match("\{(.*?)\}kml", root.tag).group(1)
    ns = {"def": namespace}

    # Define coordinates RegEx
    coord_ex = "(-?\d+\.\d+),"
    heig_ex = "(\d+)"
    regex = coord_ex + coord_ex + heig_ex

    all_coords = []  # Collect all coordinates for min/max calc

    # Find coordinates
    for i in root.findall(".//def:Placemark", ns):
        name = i.find("def:name", ns).text
        coord = i.find(".//def:coordinates", ns)
        if not coord is None:
            coord = coord.text.strip()
            coord = re.findall(regex, coord)

            # Save data
            for long, lat, heig in coord:
                all_coords.append((float(lat), float(long)))

    # Compute min/max from KML coords
    lats = [lat for lat, long in all_coords]
    longs = [long for lat, long in all_coords]
    lat_min, lat_max = (min(lats)), (max(lats)) + 0.00001
    long_min, long_max = (min(longs)), (max(longs)) + 0.00001

    # Grid + polygon check
    grid_size = 0.000008
    lat_range = np.arange(lat_min, lat_max, grid_size)
    long_range = np.arange(long_min, long_max, grid_size)

    polygon = np.array(all_coords)  # Use coords directly
    path = mplPath.Path(polygon)

    with open("inside_points.txt", "w") as inside_points:
        for i in long_range:
            if k % 2 == 0:
                for j in lat_range:
                    inside = path.contains_points([[j, i]])
                    k += 1
                    if inside == True:
                        points_list.append((j, i))
                        inside_points.write(f"{i},{j},{10}\n")
            else:
                for j in reversed(lat_range):
                    inside = path.contains_points([[j, i]])
                    k += 1
                    if inside == True:
                        points_list.append((j, i))
                        inside_points.write(f"{i},{j},{10}\n")

    # Save waypoints to output.txt
    with open("output.txt", "w") as output_file:
        output_file.write("Longitude,Latitude,Altitude\n")
        for lat, lon in points_list:
            output_file.write(f"{lon},{lat},{10}\n")

    if len(points_list) > 0:
        points_list.append(points_list[0])
        print(f"Closed path: Added first waypoint at end to complete loop")

    return points_list


def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates"""
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    dist = math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)
    return dist


def determine_waypoint_order(waypoints, home_lat, home_lon):
    """
    Compare first and last waypoints to home.
    Returns waypoints in optimal order (closest endpoint first).
    """
    if len(waypoints) < 2:
        print("Not enough waypoints to reorder")
        return waypoints

    # Calculate distance to first and last waypoint (excluding the duplicate closing point)
    dist_to_first = calculate_distance(home_lat, home_lon, waypoints[0][0], waypoints[0][1])
    dist_to_last = calculate_distance(home_lat, home_lon, waypoints[-2][0], waypoints[-2][1])

    print(f"Distance to first waypoint: {dist_to_first:.2f}m")
    print(f"Distance to last waypoint: {dist_to_last:.2f}m")

    # If last waypoint is closer, reverse the order
    if dist_to_last < dist_to_first:
        print("Last waypoint is closer → Reversing waypoint order")
        # Reverse all waypoints except keep the closing point at the end
        waypoints_reversed = waypoints[:-1][::-1]  # Reverse without closing point
        waypoints_reversed.append(waypoints_reversed[0])  # Add closing point
        return waypoints_reversed
    else:
        print("First waypoint is closer → Using original order")
        return waypoints


def scouting(waypoints, alt, resume_from_index=-1):
    global home
    global take_off
    global gps_logger_running

    vehicle.airspeed = 0.8
    print(f"Set default/target airspeed to {vehicle.airspeed}")

    # Determine optimal waypoint order based on home location
    if resume_from_index == -1:
        # Starting fresh - optimize waypoint order
        waypoints = determine_waypoint_order(waypoints, home.lat, home.lon)
        print("Starting new mission - waypoint order optimized")
    else:
        # Resuming - keep original order and continue from last completed
        print(f"Resuming mission from waypoint {resume_from_index + 2} - skipping waypoint reordering")

    point_check = [0] * len(waypoints)

    # Mark previous waypoints as completed if resuming
    if resume_from_index >= 0:
        for i in range(resume_from_index + 1):
            point_check[i] = 1
        print(f"Marked {resume_from_index + 1} waypoints as already completed")

    # Start from the next waypoint after last completed
    start_index = max(0, resume_from_index + 1)
    j = start_index + 1  # point number for display

    total_waypoints = len(waypoints)
    print(f"Total waypoints: {total_waypoints}")
    print(f"Starting from waypoint: {start_index + 1}")

    for idx, point in enumerate(waypoints[start_index:], start=start_index):
        print(f"\n{'='*50}")
        print(f"Waypoint {j}/{total_waypoints}: {point}")
        print(f"{'='*50}")

        if not take_off:
            print(f"Take off initiated")
            arm_and_takeoff(alt)
            take_off = True

        location = LocationGlobalRelative(point[0], point[1], alt)
        vehicle.simple_goto(location)
        print(f"Going to waypoint {j}")

        # 🔧 FIX: Capture current position as starting reference
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon

        print(f"Current drone position: ({current_lat:.7f}, {current_lon:.7f})")
        print(f"Target waypoint: ({point[0]:.7f}, {point[1]:.7f})")

        # Calculate initial distance from CURRENT position to waypoint
        target_dlat = point[0] - current_lat
        target_dlon = point[1] - current_lon
        target_dist_to_point = math.sqrt(
            (target_dlat * 1.113195e5) ** 2 + (target_dlon * 1.113195e5) ** 2
        )

        print(f"Distance to waypoint {j}: {target_dist_to_point:.2f}m")

        # Safety check: if already very close, mark as reached immediately
        if target_dist_to_point < 2.0:
            print(f"Already within 2m of waypoint {j}, marking as reached")
            point_check[idx] = 1
            save_mission_progress(
                waypoint_index=idx,
                total_waypoints=total_waypoints,
                last_completed_wp={"lat": point[0], "lon": point[1], "alt": alt},
                status="IN_PROGRESS"
            )
            j += 1
            continue

        # Monitor progress to waypoint
        stuck_counter = 0
        last_distance = target_dist_to_point

        while True:
            time.sleep(1)

            # Calculate current distance to waypoint
            dlat = point[0] - vehicle.location.global_relative_frame.lat
            dlon = point[1] - vehicle.location.global_relative_frame.lon
            current_dist_to_point = math.sqrt(
                (dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2
            )

            print(f"  Distance to waypoint {j}: {current_dist_to_point:.2f}m (target: {target_dist_to_point*0.98:.2f}m)")

            # Check if drone is making progress
            if abs(last_distance - current_dist_to_point) < 0.1:
                stuck_counter += 1
                if stuck_counter >= 10:
                    print(f"⚠️ Drone appears stuck, resending goto command...")
                    vehicle.simple_goto(location)
                    stuck_counter = 0
            else:
                stuck_counter = 0

            last_distance = current_dist_to_point

            # Check if reached waypoint (98% of initial distance OR within 2 meters)
            if current_dist_to_point <= max(2.0, target_dist_to_point * 0.98):
                print(f"✓ Reached waypoint {j}")
                point_check[idx] = 1

                # Save progress after completing each waypoint
                save_mission_progress(
                    waypoint_index=idx,
                    total_waypoints=total_waypoints,
                    last_completed_wp={"lat": point[0], "lon": point[1], "alt": alt},
                    status="IN_PROGRESS"
                )
                break

        # Check if we've completed all waypoints (excluding the duplicate closing point)
        if idx == len(waypoints) - 2 and point_check[idx]:
            print("\n" + "="*50)
            print("All waypoints completed! Returning to Launch")
            print("="*50)

            # Mark mission as complete
            save_mission_progress(
                waypoint_index=idx,
                total_waypoints=total_waypoints,
                last_completed_wp={"lat": point[0], "lon": point[1], "alt": alt},
                status="COMPLETE"
            )

            # Stop GPS logger
            gps_logger_running = False
            time.sleep(1)

            vehicle.mode = VehicleMode("RTL")
            time.sleep(5)

            # Close vehicle object before exiting script
            print("Close vehicle object")
            vehicle.close()
            return

        j += 1

    # Mark mission as complete
    print("\n" + "="*50)
    print("All waypoints completed. Marking mission as COMPLETE.")
    print("="*50)

    save_mission_progress(
        waypoint_index=len(waypoints)-1,
        total_waypoints=total_waypoints,
        last_completed_wp={"lat": waypoints[-1][0], "lon": waypoints[-1][1], "alt": alt},
        status="COMPLETE"
    )

    # Stop GPS logger
    gps_logger_running = False
    time.sleep(1)

    # Return to launch
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(5)

    # Close vehicle object
    print("Close vehicle object")
    vehicle.close()


# Main execution
if __name__ == "__main__":
    # Check for command line argument for KML file
    if len(sys.argv) > 1:
        kml_file = sys.argv[1]
        print(f"Using KML file from argument: {kml_file}")
    else:
        # Backup default file
        kml_file = BASE_DIR / "mission_files" / "cricketground_full.kml"
        print(f"No KML file specified, using default: {kml_file}")

    # Parse KML and get waypoints
    waypoint_list = kml_parser(kml_file)
    time.sleep(1)  # Some delay for parsing

    # Check if we should resume from a previous mission
    last_completed_index = load_mission_progress()

    if last_completed_index >= 0:
        # Some waypoints were already visited - resume from next waypoint
        print(f"\n{'='*50}")
        print("RESUMING PREVIOUS MISSION")
        print(f"Continuing from waypoint {last_completed_index + 2}")
        print(f"{'='*50}\n")
        
        # Start scouting from the waypoint after the last completed one
        scouting(waypoint_list, altitude, resume_from_index=last_completed_index)
    else:
        # No previous progress - start fresh from waypoint 0
        print(f"\n{'='*50}")
        print("STARTING NEW MISSION")
        print(f"Beginning from waypoint 1")
        print(f"{'='*50}\n")
        
        # Start scouting from the beginning
        scouting(waypoint_list, altitude, resume_from_index=-1)
