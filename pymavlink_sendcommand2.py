from pymavlink import mavutil
import time

print("Connecting to MAVProxy on port 14560...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
master.wait_heartbeat()
print("Heartbeat received!")

# --- CRITICAL ADDITION ---
# ArduPilot will ignore commands if it doesn't have a GPS fix or EKF is not ready.
# Let's wait 5 seconds to make sure SITL is fully booted.
print("Waiting 5 seconds for EKF to settle...")
time.sleep(5)

print("Sending command: Set Mode to GUIDED")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0# GUIDED
)

print("Waiting for ACK...")
start_time = time.time()

# Wait up to 5 seconds for an ACK
while time.time() - start_time < 5.0:
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    if ack is not None:
        print(f"Received ACK: Command {ack.command}, Result: {ack.result}")
        if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack.result == 0:
                print("SUCCESS! Drone accepted the command.")
            else:
                print(f"FAILED! Drone rejected the command with error code {ack.result}")
            break
else:
    print("No ACK received. MAVProxy might be blocking the return path.")