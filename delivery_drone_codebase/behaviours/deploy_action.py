import collections
import collections.abc

# --- THE FIX FOR PYTHON 3.10+ ---
# This line restores the missing attribute so DroneKit doesn't crash
collections.MutableMapping = collections.abc.MutableMapping
# --------------------------------

from pymavlink import mavutil
import time

# --- CONFIGURATION ---


# ---------------------
SERV0_1 = 9  #  AUX 1 
SERV0_2 = 10  # AUX 2

print("Connected!")
def drop(vehicle):
    global SERV0_1, SERV0_2
    
    set_servo(vehicle, SERV0_1, 1900)
    time.sleep(5)
    set_servo(vehicle, SERV0_1, 860)
    time.sleep(5)
    set_servo(vehicle, SERV0_2, 1500)
def set_servo(vehicle, servo_number, pwm_value):
    """
    Sets the servo using the MAV_CMD_DO_SET_SERVO command.
    This bypasses the dronekit 'channel override' limit.
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
        0,       # confirmation
        servo_number,  # param1: Servo number (9)
        pwm_value,     # param2: PWM (microseconds)
        0, 0, 0, 0, 0) # param3-7 (unused)
    
    vehicle.send_mavlink(msg)
    print(f" -> Command sent: Servo {servo_number} to {pwm_value}")



def Action(vehicle):
    global SERV0_1, SERV0_2
    try:
        set_servo(vehicle,SERV0_1,860)
        time.sleep(1)
        set_servo(vehicle,SERV0_2, 1160)
        
        time.sleep(1)
        
        drop(vehicle)
        time.sleep(5)
        drop(vehicle)
        
        
    except KeyboardInterrupt:
        print("User stopped script")
