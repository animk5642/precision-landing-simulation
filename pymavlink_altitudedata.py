from pymavlink import mavutil

# Connect to the drone (e.g., via UDP, TCP, or Serial)
# Replace 'udpin:localhost:14550' with your connection string or port
master = mavutil.mavlink_connection("udpin:localhost:14560")

# Wait for a heartbeat from the drone to confirm connection
master.wait_heartbeat()
print("Connected to system (ID %s, Component ID %s)" % (master.target_system, master.target_component))

while True:
  msg = master.recv_match(
      type=["GLOBAL_POSITION_INT", "VFR_HUD"], blocking=True
  )
  if not msg:
    continue

  if msg.get_type() == "GLOBAL_POSITION_INT":
    # alt is in millimeters (MSL), relative_alt is in millimeters above home
    msl_altitude_m = msg.alt / 1000.0
    relative_altitude_m = msg.relative_alt / 1000.0
    print(f"Global Alt (MSL): {msl_altitude_m:.2f} m | Relative Alt: {relative_altitude_m:.2f} m")