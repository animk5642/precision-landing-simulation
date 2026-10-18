
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

CAMERA_TOPIC = (
    "/world/iris_runway/model/iris_with_gimbal/model/gimbal/"
    "link/pitch_link/sensor/camera/image"
)


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

        self.get_logger().info("Waiting for camera images...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            cv2.imshow("Gazebo Drone Camera", frame)
            cv2.waitKey(1)
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")


def main():
    rclpy.init()
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
