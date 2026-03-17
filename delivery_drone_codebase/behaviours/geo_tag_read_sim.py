from __future__ import print_function
import time
import json
import os



class ReceiveLocationsSimulator:
    
    def __init__(self, shared_states, filename="locations.txt", delay=1):
        """
        shared_states: Dictionary containing ONLY the "json_address"
        filename: The source .txt file containing simulated coordinates
        delay: Seconds to wait between reading lines
        """
        self.shared_states = shared_states
        # Hardcoded path as per your previous snippet
        BASE_DIR = Path(__file__).resolve().parent
        self.filename = str(BASE_DIR / "locations.txt")
        self.delay = delay
        self.i = 0
        
        # Ensure the JSON file exists with the correct structure before starting
        self._ensure_json_exists()
        
        print("INFO: Simulator started.")
        print("Reading from: {}".format(self.filename))
        print("Updating JSON: {}".format(self.shared_states["json_address"]))

    def _ensure_json_exists(self):
        """Creates the JSON file if it is missing or empty."""
        path = self.shared_states["json_address"]
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            initial_structure = {
                "i": 0,
                "received_geotags": []
            }
            with open(path, "w") as f:
                json.dump(initial_structure, f, indent=4)

    def read(self):
        """
        Main loop: Reads from text file and updates the JSON file directly.
        """
        while True:
            try:
                # 1. Load current state from the JSON file
                json_path = self.shared_states["json_address"]
                with open(json_path, "r") as f:
                    json_data = json.load(f)
                    self.i = json_data.get("i", 0)

                # 2. Read the source simulation text file
                with open(self.filename, 'r') as f:
                    lines = f.readlines()

                # 3. Process the next line if available
                if self.i < len(lines):
                    line = lines[self.i].strip()

                    # Skip empty lines or comments
                    if not line or line.startswith('#'):
                        self.i += 1
                        json_data["i"] = self.i
                        self.save_json(json_data)
                        continue

                    try:
                        # Expecting format: coord_id, lat, lon
                        coord_id, lat, lon = line.split(',')
                        print("SIMULATOR: New Coordinate -> {}, {}, {}".format(coord_id, lat, lon))
                    except ValueError:
                        print("SIMULATOR_ERROR: Malformed line at index {}".format(self.i))
                        self.i += 1
                        json_data["i"] = self.i
                        self.save_json(json_data)
                        continue

                    # 4. Update the JSON data
                    # We format the name as coord{i} to match your telemetry logic
                    new_tag = ["coord{}".format(self.i), float(lat), float(lon)]
                    
                    if new_tag not in json_data["received_geotags"]:
                        json_data["received_geotags"].append(new_tag)
                        print("SIMULATOR: Logged {} to JSON".format(new_tag[0]))

                    # Increment index and save back to file
                    self.i += 1
                    json_data["i"] = self.i
                    self.save_json(json_data)

                else:
                    # End of file reached, idle
                    time.sleep(self.delay)

            except FileNotFoundError:
                print("SIMULATOR_ERROR: File not found. Checking again...")
                time.sleep(5)
            except Exception as e:
                print("SIMULATOR_ERROR: {}".format(e))
                time.sleep(self.delay)

            time.sleep(self.delay)

    def save_json(self, data):
        """Writes data to the JSON address provided in shared_states."""
        with open(self.shared_states["json_address"], "w") as f:
            json.dump(data, f, indent=4)