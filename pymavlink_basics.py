from pymavlink import mavutil

print("Connecting to MAVProxy on port 14560...")
# This acts as a client, listening for data
master = mavutil.mavlink_connection('udpin:127.0.0.1:14560')

print("Waiting for heartbeat...")
master.wait_heartbeat()

print(f"SUCCESS! Connected to system {master.target_system}")

# Request the drone to send us its attitude (roll, pitch, yaw) every 1 second
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1)

while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    print(f"Drone Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f}")