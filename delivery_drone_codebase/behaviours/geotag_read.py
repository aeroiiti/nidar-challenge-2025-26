from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import serial
import threading
import math
import json


# drone_connect = "/dev/ttyACM0"
telemetry_connect = "/dev/ttyUSB0"


class ReceiveLocations:

    def __init__(self,state):
        self.ser = serial.Serial(telemetry_connect, 9600)  # Adjust port
        self.i = 0
        self.added_coord = set()
        self.json_data = None
        self.state = state
    def read(self):
        while True:
            data = self.ser.readline()
            
            filtered_data = data.decode('utf-8')
            # print(filtered_data)
            try:
                with open(self.state["json_address"], "r") as tags:
                            self.json_data = json.load(tags)
                            self.i = self.json_data["i"]
            except FileNotFoundError:
                        print("received_geotags.json not found")
                        return
            try:
                coord, lat, lon = filtered_data.split(',')
                coord = int(coord)
                print(coord,lat,lon)
                print(type(coord))
            except ValueError as ve:
                continue
            
            if [f"coord{self.i}",float(lat), float(lon)] not in self.json_data["received_geotags"]:
                    print([f"coord{self.i}",float(lat), float(lon)])
                    
                    self.json_data["received_geotags"].append([f"coord{self.i}",float(lat), float(lon)])
                    self.json_data["geo_taged_positions"].append([f"coord{self.i}",float(lat), float(lon)])

                    self.added_coord.add(coord)

                    msg = f"{self.i},ACK\n"
                    print(f"sending: {msg}")
                    self.ser.write(msg.encode("utf-8"))
                    self.i += 1
                    self.json_data["i"] = self.i
                    self.update_response(self.json_data)

                    print("Received message sent.")

            else:
                msg = f"{self.i},ACK\n"
                print(f"sending: {msg}")
                self.ser.write(msg.encode("utf-8"))
                self.i += 1
                self.json_data["i"] = self.i

                # might want to remove this (not sure)
                self.update_response(self.json_data)

            data = ""
            print("hello")
            # print(self.shared_states["geo_taged_positions"], self.i)
    def update_response(self,data):
         with open(self.state["json_address"], "w") as f:
        # 2. ADDED indent=4 for proper JSON formatting
            json.dump(data, f, indent=4)
