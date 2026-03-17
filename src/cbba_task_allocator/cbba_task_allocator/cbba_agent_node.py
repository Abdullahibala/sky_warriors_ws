import math
from typing import Optional

import rclpy
from rclpy.node import Node

from skyw_interfaces.msg import Task, Bid, Award, TaskStatus, AgentState
from cbba_task_allocator.constants import (
    AVAIL_IDLE,
    AVAIL_EXECUTING,
    HEALTH_CRITICAL,
)

class CbbaAgentNode(Node):
    def __init__(self):
        super().__init__('cbba_agent_node')

        self.declare_parameter('agent_id', 1)
        self.declare_parameter('capability_mask', 0)
        self.declare_parameter('cruise_speed', 3.0)
        self.declare_parameter('auto_complete_s', 0.0)

        self.declare_parameter('task_topic', '/cbba/tasks')
        self.declare_parameter('bid_topic', '/cbba/bids')
        self.declare_parameter('award_topic', '/cbba/awards')
        self.declare_parameter('agent_state_topic', '/cbba/agent_state')
        self.declare_parameter('task_status_topic', '/cbba/task_status')

        self.agent_id = int(self.get_parameter('agent_id').value)
        self.capability_mask = int(self.get_parameter('capability_mask').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.auto_complete_s = float(self.get_parameter('auto_complete_s').value)

        self.task_topic = self.get_parameter('task_topic').value
        self.bid_topic = self.get_parameter('bid_topic').value
        self.award_topic = self.get_parameter('award_topic').value
        self.agent_state_topic = self.get_parameter('agent_state_topic').value
        self.task_status_topic = self.get_parameter('task_status_topic').value

        self.bid_pub = self.create_publisher(Bid, self.bid_topic, 10)
        self.status_pub = self.create_publisher(TaskStatus, self.task_status_topic, 10)

        self.create_subscription(Task, self.task_topic, self.task_cb, 10)
        self.create_subscription(Award, self.award_topic, self.award_cb, 10)
        self.create_subscription(AgentState, self.agent_state_topic, self.agent_state_cb, 10)

        self.last_state: Optional[AgentState] = None
        self.active_task_id = 0
        self.active_task_deadline = 0.0

        self.timer = self.create_timer(0.2, self.timer_cb)
        self.get_logger().info('CBBA agent node ready')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def agent_state_cb(self, msg: AgentState) -> None:
        if int(msg.agent_id) != self.agent_id:
            return
        self.last_state = msg

    def task_cb(self, msg: Task) -> None:
        bid = Bid()
        bid.task_id = int(msg.task_id)
        bid.agent_id = int(self.agent_id)
        bid.task_version = int(msg.task_version)

        feasible, score, eta, cost = self.score_task(msg)
        bid.feasible = 1 if feasible else 0
        bid.score = float(score)
        bid.eta_seconds = float(eta)
        bid.cost = float(cost)

        self.bid_pub.publish(bid)

    def award_cb(self, msg: Award) -> None:
        assigned = [int(a) for a in msg.assigned_agent_ids]
        if self.agent_id not in assigned and int(msg.winner_agent_id) != self.agent_id:
            return

        self.active_task_id = int(msg.task_id)
        self.active_task_deadline = 0.0

        status = TaskStatus()
        status.task_id = int(msg.task_id)
        status.agent_id = int(self.agent_id)
        status.status = 0
        status.info = 'executing'
        status.update_time = self._now()
        self.status_pub.publish(status)

        if self.auto_complete_s > 0.0:
            self.active_task_deadline = self._now() + self.auto_complete_s

    def timer_cb(self) -> None:
        if self.auto_complete_s <= 0.0:
            return
        if self.active_task_id == 0:
            return
        if self._now() < self.active_task_deadline:
            return

        status = TaskStatus()
        status.task_id = int(self.active_task_id)
        status.agent_id = int(self.agent_id)
        status.status = 1
        status.info = 'auto-complete'
        status.update_time = self._now()
        self.status_pub.publish(status)

        self.active_task_id = 0
        self.active_task_deadline = 0.0

    def score_task(self, msg: Task):
        if self.last_state is None:
            return False, 0.0, 0.0, 0.0

        if int(self.last_state.health_state) == HEALTH_CRITICAL:
            return False, 0.0, 0.0, 0.0

        if int(self.last_state.availability) != AVAIL_IDLE:
            return False, 0.0, 0.0, 0.0

        required = int(msg.required_capability)
        if (self.capability_mask & required) != required:
            return False, 0.0, 0.0, 0.0

        target_valid = not (math.isnan(msg.target_x) or math.isnan(msg.target_y))
        if target_valid:
            dx = msg.target_x - self.last_state.x
            dy = msg.target_y - self.last_state.y
            distance = math.hypot(dx, dy)
        else:
            distance = 0.0

        eta = distance / max(self.cruise_speed, 0.1)

        score = float(msg.priority) * 10.0
        score -= distance

        if self.last_state.battery_pct < 20.0:
            score -= (20.0 - self.last_state.battery_pct) * 2.0

        return True, score, eta, distance


def main():
    rclpy.init()
    node = CbbaAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
