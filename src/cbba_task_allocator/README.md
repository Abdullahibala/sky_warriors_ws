CBBA task allocation nodes for leader-assisted swarm coordination.

Overview
- Leader announces tasks and awards winners.
- Agents bid based on local state and capabilities.
- Executor converts awards into formation actions.

Packages and Nodes
- cbba_leader_node: task announcer and awarder
- cbba_agent_node: bidder per UAV
- agent_state_node: publishes AgentState for each UAV
- task_adapter_qr: converts decoded QR JSON to Task
- task_executor_node: executes awarded tasks (formation or wait)

Simulation Setup (GZ Harmonic + PX4)
Use your existing simulation scripts under skyw_simulation. Example order:
1) Start Gazebo and PX4 SITL for 3 UAVs
2) Start Micro-XRCE-DDS agent (if needed by PX4 bridge)
3) Start QR detection node on the leader UAV
4) Start CBBA nodes

Note: the exact sim command depends on your chosen script in skyw_simulation/new-method.

Build
1) colcon build --packages-select skyw_interfaces cbba_task_allocator
2) source install/setup.bash

Run (example for 3 UAVs)

Leader (UAV 1, camera)
1) ros2 run cbba_task_allocator agent_state_node --ros-args -p agent_id:=1 -p px4_ns:='' -p capability_mask:=1
2) ros2 run cbba_task_allocator cbba_leader_node --ros-args -p agent_id:=1
3) ros2 run cbba_task_allocator cbba_agent_node --ros-args -p agent_id:=1 -p capability_mask:=1
4) ros2 run cbba_task_allocator task_executor_node --ros-args -p agent_id:=1 -p drone_count:=3
5) ros2 run cbba_task_allocator task_adapter_qr --ros-args -p input_topic:=/qr_decoded

Follower (UAV 2)
1) ros2 run cbba_task_allocator agent_state_node --ros-args -p agent_id:=2 -p px4_ns:='/px4_1' -p capability_mask:=0
2) ros2 run cbba_task_allocator cbba_agent_node --ros-args -p agent_id:=2 -p capability_mask:=0
3) ros2 run cbba_task_allocator task_executor_node --ros-args -p agent_id:=2 -p drone_count:=3

Follower (UAV 3)
1) ros2 run cbba_task_allocator agent_state_node --ros-args -p agent_id:=3 -p px4_ns:='/px4_2' -p capability_mask:=0
2) ros2 run cbba_task_allocator cbba_agent_node --ros-args -p agent_id:=3 -p capability_mask:=0
3) ros2 run cbba_task_allocator task_executor_node --ros-args -p agent_id:=3 -p drone_count:=3

QR Detection
1) ros2 run skyw_detection qrcode_detector

Notes
- capability_mask for leader uses CAP_CAMERA (bit 0) => value 1.
- task_executor_node only acts on awards where agent_id == winner_agent_id.
- task_adapter_qr expects JSON payloads compatible with your existing QR format.
