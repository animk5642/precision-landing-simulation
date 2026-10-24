from pymavlink import mavutil
import time

print("Connecting to MAVProxy on port 14560...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
master.wait_heartbeat()
print("Heartbeat received!")

print("Waiting 5 seconds for EKF to settle...")
time.sleep(5)

GUIDED = 4  # ArduCopter mode for GUIDED

print("Sending command: Set Mode to GUIDED")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,                # Command ID
    0,                                                  # Confirmation
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Param 1 (Base Mode)
    GUIDED,                                             # Param 2 (Custom Mode)
    0, 0, 0, 0, 0                                       # Params 3-7 (Unused)
)

print("Waiting for ACK...")
start_time = time.time()
while time.time() - start_time < 5.0:
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        print(f"Received ACK: Command {ack.command}, Result: {ack.result}")
        if ack.result == 0:
            print("SUCCESS! Drone accepted the command.")
        else:
            print(f"FAILED! Result code {ack.result}")
        break
else:
    print("No ACK received within 5 seconds.")