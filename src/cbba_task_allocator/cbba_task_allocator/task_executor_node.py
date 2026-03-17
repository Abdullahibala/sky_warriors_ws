import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from skyw_interfaces.msg import Task, Award, TaskStatus
from skyw_swarm.action import SetFormation
from cbba_task_allocator.constants import TASK_FORMATION_CHANGE, TASK_WAIT


class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')

        self.declare_parameter('agent_id', 1)
        self.declare_parameter('drone_count', 3)
        self.declare_parameter('default_wait_s', 5.0)
        self.declare_parameter('task_topic', '/cbba/tasks')
        self.declare_parameter('award_topic', '/cbba/awards')
        self.declare_parameter('task_status_topic', '/cbba/task_status')
        self.declare_parameter('formation_action_name', 'set_formation')

        self.agent_id = int(self.get_parameter('agent_id').value)
        self.drone_count = int(self.get_parameter('drone_count').value)
        self.default_wait_s = float(self.get_parameter('default_wait_s').value)

        self.task_topic = self.get_parameter('task_topic').value
        self.award_topic = self.get_parameter('award_topic').value
        self.task_status_topic = self.get_parameter('task_status_topic').value
        self.formation_action_name = self.get_parameter('formation_action_name').value

        self.tasks = {}
        self.active_task_id = 0
        self.wait_timer = None

        self.status_pub = self.create_publisher(TaskStatus, self.task_status_topic, 10)
        self.create_subscription(Task, self.task_topic, self.task_cb, 10)
        self.create_subscription(Award, self.award_topic, self.award_cb, 10)

        self.formation_client = ActionClient(self, SetFormation, self.formation_action_name)

        self.get_logger().info('Task executor node ready')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def task_cb(self, msg: Task) -> None:
        self.tasks[int(msg.task_id)] = msg

    def award_cb(self, msg: Award) -> None:
        if int(msg.winner_agent_id) != self.agent_id:
            return

        task_id = int(msg.task_id)
        task = self.tasks.get(task_id)
        if task is None:
            self.get_logger().warn(f'Award received for unknown task {task_id}')
            return

        if self.active_task_id != 0:
            self.get_logger().warn('Executor busy, ignoring new award')
            return

        self.active_task_id = task_id
        self.publish_status(task_id, 0, 'executing')

        if int(task.task_type) == TASK_FORMATION_CHANGE:
            self.execute_formation(task)
        elif int(task.task_type) == TASK_WAIT:
            self.execute_wait(task)
        else:
            self.publish_status(task_id, 2, 'unsupported task type')
            self.active_task_id = 0

    def execute_formation(self, task: Task) -> None:
        if not self.formation_client.wait_for_server(timeout_sec=2.0):
            self.publish_status(int(task.task_id), 2, 'formation action not available')
            self.active_task_id = 0
            return

        goal = SetFormation.Goal()
        goal.formation_type = task.formation_type
        goal.spacing = float(task.formation_spacing)
        goal.altitude = float(task.formation_altitude)
        goal.rotation = float(task.formation_rotation_deg)
        goal.drone_count = int(self.drone_count)

        send_future = self.formation_client.send_goal_async(goal)
        send_future.add_done_callback(self._formation_goal_cb)

    def _formation_goal_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status(self.active_task_id, 2, 'formation goal rejected')
            self.active_task_id = 0
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._formation_result_cb)

    def _formation_result_cb(self, future) -> None:
        result = future.result().result
        if result.success:
            self.publish_status(self.active_task_id, 1, 'formation complete')
        else:
            self.publish_status(self.active_task_id, 2, 'formation failed')
        self.active_task_id = 0

    def execute_wait(self, task: Task) -> None:
        wait_s = self.default_wait_s
        if task.deadline_time > 0.0 and task.created_time > 0.0:
            wait_s = max(0.0, float(task.deadline_time - task.created_time))
        self.wait_timer = self.create_timer(wait_s, self._wait_done_cb)

    def _wait_done_cb(self) -> None:
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None
        self.publish_status(self.active_task_id, 1, 'wait complete')
        self.active_task_id = 0

    def publish_status(self, task_id: int, status: int, info: str) -> None:
        msg = TaskStatus()
        msg.task_id = int(task_id)
        msg.agent_id = int(self.agent_id)
        msg.status = int(status)
        msg.info = str(info)
        msg.update_time = self._now()
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TaskExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
