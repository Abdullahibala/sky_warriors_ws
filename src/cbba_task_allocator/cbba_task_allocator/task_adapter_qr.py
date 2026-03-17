import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from skyw_interfaces.msg import Task
from cbba_task_allocator.constants import (
    TASK_FORMATION_CHANGE,
    TASK_WAIT,
    CAP_CAMERA,
)


class TaskAdapterQr(Node):
    def __init__(self):
        super().__init__('task_adapter_qr')

        self.declare_parameter('agent_id', 1)
        self.declare_parameter('input_topic', '/qr_decoded')
        self.declare_parameter('task_inbox_topic', '/cbba/task_inbox')
        self.declare_parameter('required_agents', 3)

        self.agent_id = int(self.get_parameter('agent_id').value)
        self.input_topic = self.get_parameter('input_topic').value
        self.task_inbox_topic = self.get_parameter('task_inbox_topic').value
        self.required_agents = int(self.get_parameter('required_agents').value)

        self.task_pub = self.create_publisher(Task, self.task_inbox_topic, 10)
        self.create_subscription(String, self.input_topic, self.input_cb, 10)

        self.next_task_id = 1
        self.get_logger().info('Task adapter (QR) ready')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def input_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        task = Task()
        task.task_id = int(self.next_task_id)
        self.next_task_id += 1

        task.task_version = 1
        task.task_type = TASK_FORMATION_CHANGE
        task.priority = 100
        task.created_time = self._now()
        task.deadline_time = 0.0

        task.target_x = float('nan')
        task.target_y = float('nan')
        task.target_z = float('nan')
        task.target_yaw = float('nan')

        task.formation_type = ''
        task.formation_spacing = 0.0
        task.formation_altitude = 0.0
        task.formation_rotation_deg = 0.0

        task.required_agents = int(self.required_agents)
        task.required_capability = 0

        gorev = data.get('gorev', {})
        formasyon = gorev.get('formasyon', {})

        if formasyon.get('aktif'):
            task.task_type = TASK_FORMATION_CHANGE
            task.formation_type = str(formasyon.get('tip', 'v')).lower()
            task.formation_spacing = float(formasyon.get('spacing', 4.0))
            task.formation_altitude = float(gorev.get('irtifa_degisim', {}).get('deger', 20.0))
            task.formation_rotation_deg = float(formasyon.get('rotation_deg', 0.0))
        else:
            task.task_type = TASK_WAIT
            task.priority = 20

        self.task_pub.publish(task)


def main():
    rclpy.init()
    node = TaskAdapterQr()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
