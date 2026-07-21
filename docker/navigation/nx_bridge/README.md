# NX Bridge Assets

These ROS2 nodes are intended to run on the NX side as a thin adapter layer for
the `unitree-go2-nx-bridge` blueprint running on `pusai`.

Provided assets:

- `odom_to_pose.py`: converts `nav_msgs/Odometry` to `geometry_msgs/PoseStamped`
- `sport_state_to_pose.py`: adapts Go2 `SportModeState` directly to an isolated
  `geometry_msgs/PoseStamped` topic
- `cyclonedds_dual_network.xml`: connects the Go2 robot bus on `eth0` and pusai
  on `wlan0` from one adapter process
- `cyclonedds_robot_bus.xml`: connects a read-only Go2 state process only to the
  robot controller on `eth0`
- `topic_relay.py`: generic relay for lidar or camera topics
- `cmd_vel_relay.py`: disabled-by-default, rate-safe `geometry_msgs/Twist` relay for
  bench validation; it clamps velocity and publishes a zero command after a timeout
- `guarded_cmd_to_udp.py`: forwards only the guarded command topic to a local UDP
  socket from the Wi-Fi DDS participant
- `udp_to_sport_request.py`: disabled-by-default Go2 robot-bus gateway that emits
  only `MOVE` and `STOPMOVE` Sport API requests
- `start_nx_bridge.sh`: idempotently starts/stops the NX data relays and can start
  the control nodes with both movement gates still disabled

Suggested contracts:

- `/registered_scan`: `sensor_msgs/PointCloud2`
- `/odom`: `geometry_msgs/PoseStamped`
- `/color_image`: `sensor_msgs/Image`
- `/cmd_vel`: `geometry_msgs/Twist`

## One-command startup

Inside the NX Foxy container, start the verified data relays with:

```bash
cd /workspace/projects/dimos
./start_nx_bridge.sh
```

It starts the MID360 Livox driver in PointCloud2 mode, the pointcloud relay,
and the two odom UDP adapters. The default driver uses the existing built
workspace at `/workspace/tmp/livox_ros_driver2_clean_ws` and configures
`192.168.123.18 -> 192.168.123.101`, and remaps `/livox/lidar` to
`/registered_scan`.
Inspect or stop tracked processes with:

```bash
./start_nx_bridge.sh status
./start_nx_bridge.sh verify
./start_nx_bridge.sh stop
```

`./start_nx_bridge.sh --with-control` starts the three control processes, but
does not enable either movement gate.

For a bandwidth-constrained NX-to-pusai point cloud link, use
`topic_relay.py` to publish a separate, rate-limited topic. The input remains
local to NX and the output can use best-effort QoS to avoid reliable-DDS
fragment retransmissions over Wi-Fi:

```bash
python3 topic_relay.py --ros-args \
  -p input_topic:=/registered_scan \
  -p output_topic:=/dimos/registered_scan \
  -p max_rate_hz:=1.0 \
  -p output_reliability:=best_effort
```

Keep the original `/registered_scan` unchanged for local NX consumers. Point
DimOS at `/dimos/registered_scan` only after confirming the relay output is
stable on pusai.

The Go2 state bus is separate from the NX-to-pusai Wi-Fi link. For this NX it is
available on `eth0` with the robot controller (`192.168.123.161`), while pusai
is on `wlan0` (`192.168.1.196`). A single DDS participant is not sufficient for
reliable routing between both interfaces. Use two processes with local UDP:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/projects/dimos/cyclonedds_robot_bus.xml
python3 sport_state_to_udp.py

# In a second terminal, use the existing wlan0-to-pusai configuration.
export CYCLONEDDS_URI=file:///workspace/cyclonedds_ws/cyclonedds.xml
python3 udp_to_pose.py --ros-args -p output_topic:=/dimos/odom -p frame_id:=odom
```

`cmd_vel_relay.py` defaults to `/dimos/cmd_vel -> /dimos/cmd_vel_guarded`, with
`enabled:=false`, `max_linear_x:=0.15`, `max_linear_y:=0.10`,
`max_angular_z:=0.35`, and `command_timeout_sec:=0.25`. It does not target the
robot's `/cmd_vel` topic by default:

```bash
python3 cmd_vel_relay.py
```

## Go2 movement control chain

The real control path deliberately uses two DDS participants, bridged only by a
loopback UDP socket. This avoids making a single DDS participant discover both
the Pusai Wi-Fi network and the Go2 controller network:

```text
Pusai /dimos/cmd_vel
  -> NX cmd_vel_relay.py (/dimos/cmd_vel_guarded, disabled by default)
  -> NX guarded_cmd_to_udp.py (127.0.0.1:39101)
  -> NX udp_to_sport_request.py (disabled by default, robot eth0 DDS)
  -> Go2 /api/sport/request
```

The gateway never sends `StandUp`, `RecoveryStand`, or `BalanceStand`. It only
sends `MOVE` while receiving non-zero guarded commands, and `STOPMOVE` on a
zero command, timeout, or explicit disable.

Do not start `unitree_driver`, `unitree_control`, or `go2_twist_bridge_py` for
this chain. The first two contain automatic stand-up behavior and the latter
does not implement a timeout stop.

Start all three processes with both gates disabled. A person must be next to
the physical hardware stop before enabling either gate:

```bash
# Terminal A: NX Wi-Fi DDS participant, receives only Pusai traffic.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/cyclonedds_ws/cyclonedds.xml
python3 cmd_vel_relay.py
python3 guarded_cmd_to_udp.py

# Terminal B: NX robot-bus DDS participant, talks only to eth0 / Go2.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/projects/dimos/cyclonedds_robot_bus.xml
python3 udp_to_sport_request.py
```

With all processes running, both gates remain closed. Enable only after a
stationary no-motion check and with the Go2 already manually standing:

```bash
ros2 param set /nx_cmd_vel_relay enabled true
ros2 param set /udp_to_sport_request enabled true
```

Disable either gate to emit `STOPMOVE` immediately:

```bash
ros2 param set /udp_to_sport_request enabled false
ros2 param set /nx_cmd_vel_relay enabled false
```

Keep `/dimos/cmd_vel` isolated until the navigation chain and this explicit
two-gate control validation are ready for supervised hardware testing.
