# from pymavlink import mavutil
# import time

# print("Connecting to MAVProxy on port 14560...")
# master = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
# master.wait_heartbeat()
# print("Heartbeat received!")

# # 1. Define the command (Set Mode)
# # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
# # ArduCopter mode GUIDED = 4
# mode_id = 0

# print("Sending command: Set Mode to GUIDED")
# master.mav.set_mode_send(
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id
# )

# # 2. Wait for the drone to acknowledge the command
# print("Waiting for ACK...")
# while True:
#     ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
#     if ack is None:
#         print("No ACK received, command failed.")
#         break
#     if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
#         print(f"Command Accepted! Drone result: {ack.result}")
#         break