# CBBA ROS2 Implementation Plan for TEKNOFEST Swarm UAV (2026)
**Goal**: Implement a fully distributed CBBA (Consensus-Based Bundle Algorithm) node in ROS2 for dynamic task allocation. Integrate with your existing leader-follower (ID-based) on Jetson drones. Must support all competition cases from the spec (Dynamic Swarm + Semi-Autonomous missions).

**Assumptions about your workspace**:
- ROS2 Humble/Iron/Jazzy (colcon workspace)
- Existing swarm nodes (leader selection by ID, QR decoder node, camera topics, PX4/MAVSDK or Crazyflie bridge, Gazebo sim)
- Python preferred (faster prototyping on Jetson)
- Topics already available: /uav_id, /uav_pose, /battery, /qr_decoded (JSON), /colored_zones

## 1. Create the ROS2 Package (Do this first)
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python cbba_task_allocator --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs
Add to package.xml:
XML<depend>numpy</depend>
<depend>rclpy</depend>
<!-- custom msg package if you make one -->
2. Custom ROS2 Messages (Critical for tasks/bids)
Create msg/ folder and these files (standard ROS2 custom msg tutorial):

Task.msg: id, type (formation|pitch|roll|altitude|remove|rejoin|maneuver|wait), priority, target_pose, color_zone, duration
Bid.msg: agent_id, task_id, score, bundle_path (list of tasks)
Bundle.msg: agent_id, tasks (Task[]), winner
SwarmCommand.msg: for assigned actions to followers

Build with colcon build --packages-select cbba_task_allocator and source.
3. Port CBBA Core Logic (Use this excellent open-source base)
Base: https://github.com/zehuilu/CBBA-Python (pure Python, supports time windows, heterogeneous agents — perfect for UAVs)
Steps for AI:

Copy lib/CBBA.py, lib/HelperLibrary.py, WorldInfo.py into cbba_task_allocator/cbba/
Modify CBBA.solve() to be class-based and ROS2-friendly (accept tasks dynamically from topics instead of JSON)
Add UAV-specific scoring: score = (task_value / travel_distance) + battery_factor + proximity_bonus - collision_risk
Keep bundle-building + consensus phases exactly as in the repo (depth-limited search, bid computation, conflict resolution)

4. Main ROS2 Node: TaskAllocatorNode (Python)
Create cbba_task_allocator/cbba_task_allocator/task_allocator_node.py
Key structure:
Pythonclass TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('cbba_allocator')
        self.my_id = ...  # from /uav_id or param
        self.is_leader = (self.my_id == min_id)  # your existing leader-follower
        self.cbba_solver = CBBA(...) 
        self.bundle = []
        self.bids = {}  # neighbor bids

    # Subscriptions
    self.create_subscription(QRDecodedMsg, '/qr_decoded', self.qr_callback, 10)
    self.create_subscription(PoseArray, '/neighbor_poses', self.pose_callback, 10)
    self.create_subscription(BatteryState, '/battery', self.battery_callback, 10)
    self.create_subscription(BidMsg, '/bids', self.bid_callback, 10)  # peer-to-peer

    # Timer for consensus rounds (every 0.5–1s)
    self.timer = self.create_timer(0.5, self.consensus_round)
5. Implement the 5 Competition-Specific Cases
Case 1: QR Decoding Task Allocation (Dynamic Mission – Section 5.1.2)

On /qr_decoded callback: parse JSON → create Task list (formation_change, pitch_maneuver, remove_member, etc.)
Leader triggers auction → all agents bid → CBBA consensus
Output: assign “decode” task to best camera UAV

Case 2: Swarm Member Removal / Addition (Dispatch & Land in Red/Blue Zone)

Task type = "remove" with target_color_zone and UAV_ID
Scoring: proximity to detected zone (from camera) + battery
Allocated UAV: land precisely → re-arm → rejoin via new auction

Case 3: Maneuver Execution (Pitch/Roll/Yaw + Formation Rotation – Figures 4-5)

Task type = "maneuver" with angle
Allocate sub-roles if heterogeneous (e.g., best sensor UAV leads tilt)
After consensus, publish synchronized velocity commands

Case 4: Failsafe & Dynamic Reallocation (GCS Loss / UAV Failure – Sections 5.4-5.5)

On heartbeat timeout or low battery: auto-remove faulty agent from bundle
Re-run CBBA with remaining agents (distributed, no GCS needed)

Case 5: Priority-Based Handling

Add priority field in Task.msg
Modify scoring: higher priority tasks get bonus in bid calculation (ensures remove_member before next QR)