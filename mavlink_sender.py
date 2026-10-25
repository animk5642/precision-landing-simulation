from pymavlink import mavutil
import math
import time

class MavlinkSender:
    def __init__(self, connection_url='udpin:127.0.0.1:14560', marker_size=0.3):
        self.marker_size = marker_size
        
        # Add the flip parameters here
        self.flip_x = False
        self.flip_y = False
        self.swap_xy = False

        print("Connecting to MAVProxy...")
        self.mav = mavutil.mavlink_connection(connection_url)
        self.mav.wait_heartbeat(timeout=15)
        print(f"MAVLink Connected to System {self.mav.target_system}!")

    def send_target(self, x_cam, y_cam, z_cam):
        # 1. The Math
        angle_x = math.atan2(x_cam, z_cam)
        angle_y = math.atan2(y_cam, z_cam)

        # 2. Apply the flips
        if self.swap_xy:
            angle_x, angle_y = angle_y, angle_x
        if self.flip_x:
            angle_x = -angle_x
        if self.flip_y:
            angle_y = -angle_y

        # 3. Distance & Size
        distance = float(math.sqrt(x_cam**2 + y_cam**2 + z_cam**2))
        size_rad = 2.0 * math.atan2(self.marker_size / 2.0, z_cam)

        # --- ADDED PRINT STATEMENT HERE ---
        print(f"Distances -> Forward: {-y_cam:.2f}m, Right: {x_cam:.2f}m, Down: {z_cam:.2f}m")
        print(f"Angles -> X: {angle_x:.3f}, Y: {angle_y:.3f}")
        # ----------------------------------

        # 4. The MAVLink Send
        self.mav.mav.landing_target_send(
            int(time.time() * 1e6), 
            0, 
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            angle_x, angle_y, distance, size_rad, size_rad,
            0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0],
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL, 
            0
        )