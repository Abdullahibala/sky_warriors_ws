import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition

from skyw_interfaces.msg import AgentState
from cbba_task_allocator.constants import (
    HEALTH_OK,
    AVAIL_IDLE,
)


class AgentStateNode(Node):
    def __init__(self):
        super().__init__('agent_state_node')

        self.declare_parameter('agent_id', 1)
        self.declare_parameter('px4_ns', '')
        self.declare_parameter('battery_topic', '')
        self.declare_parameter('battery_default', 100.0)
        self.declare_parameter('capability_mask', 0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('agent_state_topic', '/cbba/agent_state')

        self.agent_id = int(self.get_parameter('agent_id').value)
        self.px4_ns = self.get_parameter('px4_ns').value
        self.battery_topic = self.get_parameter('battery_topic').value
        self.battery_default = float(self.get_parameter('battery_default').value)
        self.capability_mask = int(self.get_parameter('capability_mask').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.agent_state_topic = self.get_parameter('agent_state_topic').value

        self.battery_pct = self.battery_default
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)

        px4_topic = '/fmu/out/vehicle_local_position_v1'
        if self.px4_ns:
            px4_topic = f'{self.px4_ns}/fmu/out/vehicle_local_position_v1'

        self.create_subscription(VehicleLocalPosition, px4_topic, self.position_cb, 10)

        if self.battery_topic:
            self.create_subscription(Float32, self.battery_topic, self.battery_cb, 10)

        self.state_pub = self.create_publisher(AgentState, self.agent_state_topic, 10)

        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self.publish_state)

        self.get_logger().info('Agent state node ready')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def battery_cb(self, msg: Float32) -> None:
        self.battery_pct = float(msg.data)

    def position_cb(self, msg: VehicleLocalPosition) -> None:
        self.position = (float(msg.x), float(msg.y), float(-msg.z))
        self.velocity = (float(msg.vx), float(msg.vy), float(msg.vz))

    def publish_state(self) -> None:
        state = AgentState()
        state.agent_id = int(self.agent_id)
        state.x = self.position[0]
        state.y = self.position[1]
        state.z = self.position[2]
        state.yaw = 0.0
        state.vx = self.velocity[0]
        state.vy = self.velocity[1]
        state.vz = self.velocity[2]
        state.battery_pct = float(self.battery_pct)
        state.health_state = HEALTH_OK
        state.availability = AVAIL_IDLE
        state.current_task_id = 0
        state.capability_mask = int(self.capability_mask)
        state.last_update_time = self._now()

        self.state_pub.publish(state)


def main():
    rclpy.init()
    node = AgentStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
