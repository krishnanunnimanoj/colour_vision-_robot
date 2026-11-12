import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.color_pub = self.create_publisher(
            String,
            '/detected_color',
            10
        )

        self.get_logger().info('Colour detector started')

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = {
            'red': (
                cv2.inRange(hsv, np.array([0, 100, 80]),
                            np.array([10, 255, 255])) |
                cv2.inRange(hsv, np.array([170, 100, 80]),
                            np.array([180, 255, 255]))
            ),
            'green': cv2.inRange(
                hsv, np.array([35, 70, 60]),
                np.array([85, 255, 255])
            ),
            'blue': cv2.inRange(
                hsv, np.array([90, 70, 60]),
                np.array([135, 255, 255])
            ),
            'yellow': cv2.inRange(
                hsv, np.array([20, 100, 100]),
                np.array([35, 255, 255])
            )
        }

        detected_color = None
        largest_area = 0

        for color, mask in masks.items():

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                if area > largest_area and area > 500:
                    largest_area = area
                    detected_color = color

        output = String()

        if detected_color is not None:
            output.data = detected_color
        else:
            output.data = 'none'

        self.color_pub.publish(output)

def main(args=None):
    rclpy.init(args=args)

    node = ColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

