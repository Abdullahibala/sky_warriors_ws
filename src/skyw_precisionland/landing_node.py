import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from px4_msgs.msg import TrajectorySetpoint

class PrecisionLanding(Node):

    def __init__(self):
        super().__init__('precision_landing')

        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.marker_callback,
            10)

        self.publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10)

    def marker_callback(self, msg):

        if len(msg.poses) == 0:
            return

        marker = msg.poses[0]

        x_offset = marker.position.x
        y_offset = marker.position.y

        setpoint = TrajectorySetpoint()

        setpoint.position = [
            -x_offset,
            -y_offset,
            -1.0
        ]

        self.publisher.publish(setpoint)


def main(args=None):

    rclpy.init(args=args)

    node = PrecisionLanding()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
