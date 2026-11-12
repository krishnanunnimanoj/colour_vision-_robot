import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Empty


class NavigationController(Node):

    def __init__(self):
        super().__init__('navigation_controller')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.create_subscription(
            String, '/detected_color',
            self.color_callback, 10
        )

        self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10
        )

        self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )

        self.create_subscription(
            Empty, '/return_home',
            self.home_callback, 10
        )

        self.timer = self.create_timer(
            0.1, self.control_loop
        )
        self.delete_client = self.create_client(
            DeleteEntity, '/delete_entity'
        )

        self.sample_entity_name = None

        self.x = None
        self.y = None
        self.yaw = None
        self.ranges = []

        self.state = 'IDLE'
        self.goal_x = None
        self.goal_y = None

        self.targets = {
            'red': (2.61, 2.61),
            'green': (-2.61, 2.61),
            'blue': (-2.61, -2.61),
            'yellow': (2.61, -2.61),
            'other': (2.61, -2.61)
        }

        self.target_yaws = {
            'red': math.pi / 4,
            'green': 3 * math.pi / 4,
            'blue': -3 * math.pi / 4,
            'yellow': -math.pi / 4,
            'other': -math.pi / 4
        }

        self.goal_yaw = None

        self.home_x = None
        self.home_y = None
        self.home_yaw = None

        self.get_logger().info(
            'Navigation controller ready. Waiting for colour detection.'
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(sin_yaw, cos_yaw)

        if self.home_x is None:
            self.home_x = self.x
            self.home_y = self.y
            self.home_yaw = self.yaw

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
    def color_callback(self, msg):

        color = msg.data.strip().lower()

        if color == 'none':
            return

        if self.state != 'IDLE':
            return

        if color not in self.targets:
            color = 'other'

        # Store the Gazebo sample entity name
        self.sample_entity_name = f'{color}_object'

        self.goal_x, self.goal_y = self.targets[color]
        self.goal_yaw = self.target_yaws[color]

        self.state = 'GO_TO_TARGET'

        self.get_logger().info(
            f'Detected {color}. Navigating to target.'
        )
    def home_callback(self, msg):
        if self.state == 'WAIT_FOR_HOME':
            self.goal_x = self.home_x
            self.goal_y = self.home_y
            self.state = 'RETURN_HOME'

            self.get_logger().info(
                'Return-home command received.'
            )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def delete_sample(self):

        if self.sample_entity_name is None:
            self.get_logger().warning(
                'No sample entity name available.'
            )
            self.state = 'WAIT_FOR_HOME'
            return

        if not self.delete_client.service_is_ready():
            self.get_logger().warning(
                'Gazebo delete_entity service is not available.'
            )
            self.state = 'WAIT_FOR_HOME'
            return

        request = DeleteEntity.Request()
        request.name = self.sample_entity_name

        self.state = 'DELETING_SAMPLE'

        future = self.delete_client.call_async(request)
        future.add_done_callback(self.delete_sample_callback)

        self.get_logger().info(
            f'Requesting deletion of {self.sample_entity_name}.'
        )
    def delete_sample_callback(self, future):

        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f'Sample {self.sample_entity_name} deleted successfully.'
                )
            else:
                self.get_logger().error(
                    f'Failed to delete sample: {response.status_message}'
                )

        except Exception as e:
            self.get_logger().error(
                f'Deletion request failed: {e}'
            )

        self.sample_entity_name = None
        self.state = 'WAIT_FOR_HOME'

        self.get_logger().info(
            'Waiting for return-home command.'
        )
    def control_loop(self):

        if self.state not in ('GO_TO_TARGET', 'RETURN_HOME'):
            self.stop_robot()
            return

        if self.x is None or self.yaw is None or not self.ranges:
            self.stop_robot()
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.hypot(dx, dy)

        if distance < 0.15:

            if self.state == 'RETURN_HOME':
                desired_yaw = self.home_yaw

            else:
                desired_yaw = self.goal_yaw

            yaw_error = math.atan2(
                math.sin(desired_yaw - self.yaw),
                math.cos(desired_yaw - self.yaw)
            )

            if abs(yaw_error) > 0.30:
                cmd = Twist()
                cmd.angular.z = max(
                    -0.80, min(0.80, 3.0 * yaw_error)
                )
                self.cmd_pub.publish(cmd)
                return

            self.stop_robot()

            if self.state == 'RETURN_HOME':
                self.state = 'IDLE'
                self.get_logger().info(
                    'Home reached and original orientation restored.'
                )
            else:
                self.get_logger().info(
                    'Target reached and aligned. Deleting sample.'
                )
                self.delete_sample()

            return
            # Goal attraction
        goal_distance = math.hypot(dx, dy)

        goal_x = dx / goal_distance
        goal_y = dy / goal_distance

        attraction_x = 1.5 * goal_x
        attraction_y = 1.5 * goal_y

        # Obstacle repulsion
        repulsion_x = 0.0
        repulsion_y = 0.0

        influence_distance = 0.9
        front_min = float('inf')

        for i, r in enumerate(self.ranges):

            if not math.isfinite(r) or r <= 0.12:
                continue

            angle = self.angle_min + i * self.angle_increment

            # Measure front clearance
            if -0.35 <= angle <= 0.35:
                front_min = min(front_min, r)

            if r < influence_distance:

                obstacle_angle = self.yaw + angle

                strength = (
                    (1.0 / r) -
                    (1.0 / influence_distance)
                ) / (r * r)

                repulsion_x -= strength * math.cos(obstacle_angle)
                repulsion_y -= strength * math.sin(obstacle_angle)

        # Limit repulsive force
        repulsion_magnitude = math.hypot(
            repulsion_x, repulsion_y
        )

        if repulsion_magnitude > 0:
            scale = min(1.5, repulsion_magnitude * 0.08)

            repulsion_x = (
                repulsion_x / repulsion_magnitude
            ) * scale

            repulsion_y = (
                repulsion_y / repulsion_magnitude
            ) * scale

        # Combine attraction and repulsion
        total_x = attraction_x + repulsion_x
        total_y = attraction_y + repulsion_y

        desired_yaw = math.atan2(total_y, total_x)

        yaw_error = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw)
        )

        cmd = Twist()

        # Steering
        cmd.angular.z = max(
            -0.7, min(0.7, 1.8 * yaw_error)
        )

        # Speed control
        
        if abs(yaw_error) > 0.5:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(0.30, 0.7 * distance)

            if front_min < 0.65:
                cmd.linear.x = min(cmd.linear.x, 0.10)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    node = NavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

