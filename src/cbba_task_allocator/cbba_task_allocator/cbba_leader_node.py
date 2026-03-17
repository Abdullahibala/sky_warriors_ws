import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from skyw_interfaces.msg import Task, Bid, Award, TaskStatus


class CbbaLeaderNode(Node):
    def __init__(self):
        super().__init__('cbba_leader_node')

        self.declare_parameter('agent_id', 1)
        self.declare_parameter('bidding_window_s', 1.0)
        self.declare_parameter('reannounce_interval_s', 2.0)
        self.declare_parameter('task_inbox_topic', '/cbba/task_inbox')
        self.declare_parameter('task_topic', '/cbba/tasks')
        self.declare_parameter('bid_topic', '/cbba/bids')
        self.declare_parameter('award_topic', '/cbba/awards')
        self.declare_parameter('task_status_topic', '/cbba/task_status')

        self.agent_id = int(self.get_parameter('agent_id').value)
        self.bidding_window_s = float(self.get_parameter('bidding_window_s').value)
        self.reannounce_interval_s = float(self.get_parameter('reannounce_interval_s').value)

        self.task_inbox_topic = self.get_parameter('task_inbox_topic').value
        self.task_topic = self.get_parameter('task_topic').value
        self.bid_topic = self.get_parameter('bid_topic').value
        self.award_topic = self.get_parameter('award_topic').value
        self.task_status_topic = self.get_parameter('task_status_topic').value

        self.task_pub = self.create_publisher(Task, self.task_topic, 10)
        self.award_pub = self.create_publisher(Award, self.award_topic, 10)

        self.create_subscription(Task, self.task_inbox_topic, self.task_inbox_cb, 10)
        self.create_subscription(Bid, self.bid_topic, self.bid_cb, 10)
        self.create_subscription(TaskStatus, self.task_status_topic, self.task_status_cb, 10)

        self.tasks: Dict[int, dict] = {}

        self.timer = self.create_timer(0.2, self.timer_cb)
        self.get_logger().info('CBBA leader node ready')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def task_inbox_cb(self, msg: Task) -> None:
        task_id = int(msg.task_id)
        now = self._now()

        if task_id not in self.tasks:
            version = 1
        else:
            version = self.tasks[task_id]['version'] + 1

        msg.task_version = int(version)

        self.tasks[task_id] = {
            'task': msg,
            'version': version,
            'announced_time': now,
            'last_announce_time': now,
            'bids': {},
            'awarded': False,
        }

        self.task_pub.publish(msg)
        self.get_logger().info(f'Announced task {task_id} v{version}')

    def bid_cb(self, msg: Bid) -> None:
        task_id = int(msg.task_id)
        if task_id not in self.tasks:
            return

        task_info = self.tasks[task_id]
        if int(msg.task_version) != int(task_info['version']):
            return

        task_info['bids'][int(msg.agent_id)] = msg

    def task_status_cb(self, msg: TaskStatus) -> None:
        task_id = int(msg.task_id)
        if task_id not in self.tasks:
            return

        if int(msg.status) == 1:
            self.tasks.pop(task_id, None)
            self.get_logger().info(f'Task {task_id} completed')
            return

        if int(msg.status) == 2:
            task_info = self.tasks[task_id]
            task_info['version'] += 1
            task_info['task'].task_version = int(task_info['version'])
            task_info['announced_time'] = self._now()
            task_info['last_announce_time'] = task_info['announced_time']
            task_info['bids'] = {}
            task_info['awarded'] = False
            self.task_pub.publish(task_info['task'])
            self.get_logger().warn(f'Task {task_id} failed, reannounced v{task_info["version"]}')

    def timer_cb(self) -> None:
        now = self._now()
        for task_id, info in list(self.tasks.items()):
            if info['awarded']:
                continue

            if now - info['announced_time'] < self.bidding_window_s:
                continue

            bids: List[Bid] = list(info['bids'].values())
            feasible_bids = [b for b in bids if int(b.feasible) == 1]
            required_agents = max(1, int(info['task'].required_agents))

            if not feasible_bids:
                if now - info['last_announce_time'] >= self.reannounce_interval_s:
                    info['version'] += 1
                    info['task'].task_version = int(info['version'])
                    info['announced_time'] = now
                    info['last_announce_time'] = now
                    info['bids'] = {}
                    self.task_pub.publish(info['task'])
                    self.get_logger().warn(f'Task {task_id} no bids, reannounced v{info["version"]}')
                continue

            feasible_bids.sort(key=lambda b: (-b.score, b.eta_seconds, b.agent_id))
            selected = feasible_bids[:required_agents]

            award = Award()
            award.task_id = int(task_id)
            award.task_version = int(info['version'])

            award.winner_agent_id = int(selected[0].agent_id)
            award.assigned_agent_ids = [int(b.agent_id) for b in selected]

            self.award_pub.publish(award)
            info['awarded'] = True
            self.get_logger().info(
                f'Awarded task {task_id} v{info["version"]} to {award.assigned_agent_ids}'
            )


def main():
    rclpy.init()
    node = CbbaLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
