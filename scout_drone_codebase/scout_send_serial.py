import serial
import time
import json

# 1. ADDED TIMEOUT=1 (Prevents the script from hanging forever)
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

time.sleep(2)

delivered = 0
new_tags = 0
temp_tag = []

def update_response(data, file_name="detected_persons.json"):
    with open(file_name, "w") as f:
        # 2. ADDED indent=4 for proper JSON formatting
        json.dump(data, f, indent=4)

def main():
    global new_tags, temp_tag, ser, delivered
    
    try:
        with open("detected_persons.json", "r") as tags:
            data = json.load(tags)
    except FileNotFoundError:
        print("detected_persons.json not found")
        return

    new_tags = len(data["geotag"])
        
    if new_tags > delivered:
        temp_tag = data["geotag"][delivered]
        coord_name = delivered
        lat = temp_tag["lat"]
        lon = temp_tag["lon"]
        
        # Ensure the message we send has a newline
        msg = f"{coord_name},{lat},{lon}\n"
        tries = 4

        while tries > 0:
            ser.write(msg.encode('utf-8'))
            print(f"Sent: {msg.strip()}")
            
            # This will now return None or empty after 1 second because of the timeout
            read_for_ack = ser.readline()

            if not read_for_ack: 
                print(f"No response. Retries left: {tries-1}")
                tries -= 1

                continue

            try:
                filtered_response = read_for_ack.decode('utf-8').strip()
                if not filtered_response:
                    tries -= 1
                    continue

                person_id, response = filtered_response.split(",")

                if response == "ACK":
                    data["geotag"][delivered]["received"] = True
                    print(f"Geotag {delivered} delivered successfully")
                    update_response(data) # This now saves with indentation
                    delivered += 1
                    break
                else:
                    # delivered+=1
                    tries -= 1

            except (ValueError, UnicodeDecodeError):
                tries -= 1
                delivered+=1
                continue

        if tries == 0:
            delivered+=1
            print(f"Failed to send geotag with id: {delivered}")

# if __name__ == "__main__":
#     while True:
#         main()
#         time.sleep(1)