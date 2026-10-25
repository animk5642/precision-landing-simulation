import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from mavlink_sender import MavlinkSender

CAM_BASE = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera'

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        # RESTORED: ROS Parameters
        self.declare_parameter('marker_size', 0.3)
        self.declare_parameter('marker_id', -1)
        
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value

        # Initialize the MAVLink helper with the correct marker size
        self.mavlink = MavlinkSender(marker_size=self.marker_size)

        self.image_sub = self.create_subscription(Image, CAM_BASE + '/image', self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, CAM_BASE + '/camera_info', self.info_callback, 10)

        self.bridge = CvBridge()
        self.cam_matrix = None
        self.dist = None
        
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.dict, cv2.aruco.DetectorParameters())
        
        half = self.marker_size / 2.0
        self.obj_pts = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)

    def info_callback(self, msg):
        self.cam_matrix = np.array(msg.k).reshape((3, 3))
        self.dist = np.array(msg.d) if len(msg.d) else np.zeros(5)
        self.destroy_subscription(self.info_sub)

    # RESTORED: Marker ID Selection
    def pick_marker(self, ids):
        flat = ids.flatten()
        if self.marker_id < 0:
            return 0
        matches = np.where(flat == self.marker_id)[0]
        return int(matches[0]) if len(matches) else None

    def image_callback(self, msg):
        if self.cam_matrix is None: return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:
            idx = self.pick_marker(ids)
            if idx is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                img_points = corners[idx][0].astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(self.obj_pts, img_points, self.cam_matrix, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                
                if ok:
                    x, y, z = tvec.flatten()
                    self.mavlink.send_target(x, y, z)

        cv2.imshow('Vision', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()