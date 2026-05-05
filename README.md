# clearpath_j100

> Custom Nav2 bringup, virtual joystick UI, and Nav2 goal panel for the Clearpath J100 (Jackal) Gazebo simulation.

---

## Demo

<video src="Screencast%20from%2005-05-2026%2002%3A21%3A04%20AM.webm" controls width="720"></video>

<video src="Screencast%20from%2005-06-2026%2001%3A03%3A30%20AM.webm" controls width="720"></video>

---

## What's in this repo

- **`src/j100_nav2_bringup/`** — ROS 2 Python package: one-command launch file that starts Gazebo, RViz, AMCL, Nav2, and the Qt goal panel in the correct order using timed sequencing.
- **`joystick_ui.py`** — standalone PyQt5 virtual joystick that publishes directly to `/j100_0001/cmd_vel` at 20 Hz.
- **`dependencies.repos`** — `vcstool` manifest that pulls in the three upstream Clearpath source packages (`clearpath_common`, `clearpath_config`, `clearpath_msgs`) from the `humble` branch.
- **Screencast `.webm` files** — demo recordings tracked with Git LFS.

Files that are NOT in the repo but are required at runtime are documented in [Section 6](#files-not-in-this-repo-must-set-up-manually).

---

## Prerequisites

| Requirement | Notes |
|---|---|
| Ubuntu 22.04 | Tested on 22.04 LTS |
| ROS 2 Humble (desktop) | Full desktop install recommended |
| Gazebo Fortress (Ignition) | Installed via `ros-humble-clearpath-gz` |
| Python 3.10+ | Ships with Ubuntu 22.04 |

Install apt dependencies:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-clearpath-gz \
  ros-humble-clearpath-viz \
  ros-humble-clearpath-nav2-demos \
  python3-pyqt5 \
  git-lfs \
  python3-vcstool
```

---

## Setup

### 5a. Clone (with LFS)

Git LFS must be initialized before cloning so that the `.webm` screencasts are fetched as real files rather than text pointers.

```bash
git lfs install
git clone git@github.com:Bigenlight/clearpath_j100.git ~/clearpath_ws
cd ~/clearpath_ws
git lfs pull   # ensure LFS objects are downloaded
```

### 5b. Import upstream Clearpath sources

```bash
cd ~/clearpath_ws
vcs import src < dependencies.repos
```

This pulls `clearpath_common`, `clearpath_config`, and `clearpath_msgs` (all `humble` branch) into `src/`.

### 5c. Install ROS dependencies

```bash
cd ~/clearpath_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5d. Create `~/clearpath/robot.yaml`

Clearpath tooling reads this file at launch time to configure the robot namespace, sensors, and velocity limits. Create it with the exact content below:

```bash
mkdir -p ~/clearpath
```

```yaml
serial_number: j100-0001
version: 0
system:
  username: administrator
  hosts:
    - hostname: cpr-j100-0001
      ip: 192.168.131.1
  ros2:
    namespace: j100_0001
    domain_id: 0
    middleware:
      implementation: rmw_fastrtps_cpp
  extras:
    ros_parameters:
      platform_velocity_controller:
        linear.x.max_velocity: 4.0
        linear.x.min_velocity: -4.0
        linear.x.max_acceleration: 20.0
        linear.x.min_acceleration: -20.0
        angular.z.max_velocity: 8.0
        angular.z.min_velocity: -8.0
        angular.z.max_acceleration: 25.0
        angular.z.min_acceleration: -25.0
platform:
  controller: ps4
  decorations:
    front_bumper:
      enabled: true
      model: default
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
      extension: 0.0
    rear_bumper:
      enabled: true
      model: default
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
      extension: 0.0
    top_plate:
      enabled: true
      model: default
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
sensors:
  lidar2d:
    - model: hokuyo_ust
      urdf_enabled: true
      launch_enabled: true
      parent: front_0_mount
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]
      ros_parameters:
        urg_node:
          angle_min: -1.5707
          angle_max: 1.5707
```

### 5e. Prepare a map (required for Nav2 localization)

Nav2 localization (AMCL) requires a pre-built occupancy map. The map is **not included in this repo** because it is environment-specific.

The launch file expects:
- `~/clearpath/maps/my_map.yaml`
- `~/clearpath/maps/my_map.pgm`

To generate one, run SLAM in the same Gazebo world (e.g., with `slam_toolbox`), then save the map:

```bash
mkdir -p ~/clearpath/maps
ros2 run nav2_map_server map_saver_cli -f ~/clearpath/maps/my_map
```

If you already have a compatible map file pair, copy them to `~/clearpath/maps/` and rename them `my_map.yaml` / `my_map.pgm`.

### 5f. Build the workspace

```bash
cd ~/clearpath_ws
colcon build --symlink-install
source install/setup.bash
```

Add the source line to your shell RC file so it persists across terminals:

```bash
echo "source ~/clearpath_ws/install/setup.bash" >> ~/.bashrc
```

### 5g. Apply the system Nav2 patch

The upstream DWB planner config causes the J100 to oscillate left-right when arriving at a goal. Two values must be patched in the system-installed file:

```
/opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml
```

Apply with:

```bash
sudo sed -i 's/yaw_goal_tolerance: 0\.3/yaw_goal_tolerance: 0.5/' \
  /opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml

sudo sed -i 's/RotateToGoal\.scale: 32\.0/RotateToGoal.scale: 16.0/' \
  /opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml
```

Verify:

```bash
grep -E "yaw_goal_tolerance|RotateToGoal.scale" \
  /opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml
```

Expected output:
```
      yaw_goal_tolerance: 0.5
      RotateToGoal.scale: 16.0
```

> **Note:** This patch must be re-applied after any `apt upgrade` that updates `ros-humble-clearpath-nav2-demos`. See [Section 8](#architecture-notes) for rationale.

---

## Files and Modifications

### `src/j100_nav2_bringup/` — package overview

A ROS 2 ament-python package that provides a single launch file and a Qt Nav2 goal UI node. It orchestrates the entire simulation stack from one command, eliminating the need to open multiple terminals or manually set an initial pose in RViz.

---

### `src/j100_nav2_bringup/launch/bringup.launch.py`

**Purpose:** Starts the full Nav2 stack in a single `ros2 launch` call, using `TimerAction` to sequence components so each one starts only after its dependencies are ready.

**Launch argument:**

| Argument | Default | Description |
|---|---|---|
| `world` | `warehouse` | Gazebo world passed to `clearpath_gz simulation.launch.py` |

**ASCII startup timeline:**

```
t=  0s  ├── clearpath_gz  simulation.launch.py  (Gazebo + robot spawner)
         ├── clearpath_viz view_navigation.launch.py  (RViz)
         │
t= 10s  ├── clearpath_nav2_demos localization.launch.py  (AMCL)
         │      map: ~/clearpath/maps/my_map.yaml
         │
t= 15s  ├── clearpath_nav2_demos nav2.launch.py  (planner + controller + BT)
         │
t= 25s  ├── ros2 topic pub --once /j100_0001/initialpose  (auto initial pose)
         │      position: (0, 0, 0)   orientation: identity
         │
t= 30s  └── nav_goal_ui node  (Qt goal panel)
```

**Key design choices vs upstream:**
- All `use_sim_time: true` — the sim clock drives every component.
- `setup_path` resolves to `$HOME/clearpath/` via `EnvironmentVariable` substitution, so no hard-coded home directory.
- Initial pose is published automatically at t=25s with identity covariance (σ_xy=0.25 m, σ_yaw=0.0685 rad), removing the manual "2D Pose Estimate" step in RViz.

---

### `src/j100_nav2_bringup/j100_nav2_bringup/nav_goal_ui.py`

**Purpose:** PyQt5 window that sends `NavigateToPose` action goals to Nav2 and displays live goal status. Launched automatically at t=30s by the bringup file.

**Action server:** `/j100_0001/navigate_to_pose`

**StatusBridge pattern:**

The ROS 2 executor spins in a background daemon thread (`SingleThreadedExecutor`). All GUI updates must happen on the Qt main thread. `StatusBridge` is a `QObject` subclass that exposes a single `pyqtSignal(str)` — `status_changed`. The ROS callbacks emit this signal; Qt's cross-thread queued connection delivers it safely to the `QLabel` on the main thread.

```
ROS executor thread          Qt main thread
─────────────────────        ──────────────────────────
_feedback_cb()         →     status_changed.emit(str)
_goal_response_cb()    →           ↓ (queued)
_result_cb()           →     lbl_status.setText(str)
```

**Saved point constants:**

| Constant | Value | Description |
|---|---|---|
| `SAVED_X` | `1.8797` | X coordinate in map frame (warehouse-specific) |
| `SAVED_Y` | `-12.0739` | Y coordinate in map frame (warehouse-specific) |
| `SAVED_QZ` | `-0.7179` | Quaternion Z component of goal heading |
| `SAVED_QW` | `0.6961` | Quaternion W component of goal heading |

These coordinates are tuned for the Clearpath warehouse Gazebo world. Edit them in `nav_goal_ui.py` for other environments.

**UI panels:**

| Panel | Controls |
|---|---|
| Saved Point | Shows coordinates; "Send to Saved Point" button sends the hardcoded goal |
| Custom Goal | Spinboxes for X (m), Y (m), Yaw (rad); "Send Custom Goal" converts yaw → quaternion |
| Status | Large bold status label (IDLE / PENDING / ACTIVE — N m left / SUCCEEDED / ABORTED / CANCELED); "Cancel Goal" button |

---

### `src/j100_nav2_bringup/package.xml` and `setup.py`

**Entry point registered by `setup.py`:**

```python
'console_scripts': [
    'nav_goal_ui = j100_nav2_bringup.nav_goal_ui:main',
]
```

This makes `nav_goal_ui` available as a `ros2 run` executable and as a `Node(executable='nav_goal_ui')` target in the launch file.

**Runtime dependencies declared in `package.xml`:**

| Dependency | Role |
|---|---|
| `rclpy` | ROS 2 Python client |
| `geometry_msgs` | `PoseStamped` |
| `nav2_msgs` | `NavigateToPose` action |
| `action_msgs` | `GoalStatus` constants |
| `python3-pyqt5` | Qt GUI toolkit |
| `clearpath_gz` | Simulation launch |
| `clearpath_viz` | RViz launch |
| `clearpath_nav2_demos` | Localization + Nav2 launches and config |
| `ros2cli`, `ros2topic` | `ExecuteProcess` call for initial pose |

---

### `joystick_ui.py`

**Purpose:** Standalone PyQt5 virtual joystick. Run independently (not through the bringup launch file) for manual teleoperation during map building or debugging.

**Key constants:**

| Constant | Value | Description |
|---|---|---|
| `CMD_VEL_TOPIC` | `/j100_0001/cmd_vel` | Publish target |
| `PUBLISH_PERIOD_MS` | `50` | 20 Hz publish rate (well under twist_mux 0.5s timeout) |
| `DEFAULT_MAX_LINEAR` | `0.5 m/s` | Slider initial value |
| `DEFAULT_MAX_ANGULAR` | `1.0 rad/s` | Slider initial value |
| `HARD_MAX_LINEAR` | `4.0 m/s` | Slider upper bound (robot.yaml controller limit) |
| `HARD_MAX_ANGULAR` | `8.0 rad/s` | Slider upper bound (robot.yaml controller limit) |
| `AXIS_SNAP_FRAC` | `0.15` | Snap to pure axis within 15% of cardinal direction |

**QoS profile:** `BEST_EFFORT` reliability, `VOLATILE` durability — matches the twist_mux subscriber.

**Threading model:** The ROS 2 node spins in a `SingleThreadedExecutor` on a background daemon thread. All publishing happens on the Qt main thread via a `QTimer` firing every 50 ms. No lock is needed because the only shared state (`_joy_x`, `_joy_y`) is written by a Qt slot and read by a Qt timer — both on the main thread.

**Safety features:**

| Feature | Behavior |
|---|---|
| Spring-centered knob | Mouse release snaps knob to center and emits `(0, 0)` |
| Focus loss zeroing | `ApplicationStateChange` event zeroes joystick on alt-tab / window deactivate |
| Emergency stop button | Spacebar shortcut or click; publishes zeros and locks out joystick; red/amber toggle styling |
| Close event | Publishes one final zero twist before ROS teardown |

**Axis convention:** Joystick up → positive `linear.x`; joystick right → negative `angular.z` (REP-103 right-hand rule: clockwise yaw = negative).

---

### `dependencies.repos`

Pulled by `vcs import src < dependencies.repos`. Content:

```yaml
repositories:
  clearpath_common:
    type: git
    url: https://github.com/clearpathrobotics/clearpath_common.git
    version: humble
  clearpath_config:
    type: git
    url: https://github.com/clearpathrobotics/clearpath_config.git
    version: humble
  clearpath_msgs:
    type: git
    url: https://github.com/clearpathrobotics/clearpath_msgs.git
    version: humble
```

These are the upstream Clearpath source packages not available as pre-built Humble debs (or where a newer source version is needed). They land in `src/clearpath_common/`, `src/clearpath_config/`, and `src/clearpath_msgs/`.

---

### Files NOT in this repo (must set up manually)

#### `~/clearpath/robot.yaml`

The Clearpath platform configuration file. All launch files resolve `setup_path` to `$HOME/clearpath/` and read this file to configure the robot namespace, sensor URDF, and velocity controller parameters.

Key fields:

| Field | Value | Effect |
|---|---|---|
| `system.ros2.namespace` | `j100_0001` | All topics, actions, and TF frames use this prefix |
| `sensors.lidar2d[0].model` | `hokuyo_ust` | Spawns a Hokuyo UST LiDAR on `front_0_mount` |
| `sensors.lidar2d[0].parent` | `front_0_mount` | Mount point on the robot URDF |
| `linear.x.max_velocity` | `4.0 m/s` | Controller hard cap (joystick slider ceiling) |
| `linear.x.min_velocity` | `-4.0 m/s` | Controller hard cap (reverse) |
| `angular.z.max_velocity` | `8.0 rad/s` | Controller hard cap (joystick slider ceiling) |
| `angular.z.min_velocity` | `-8.0 rad/s` | Controller hard cap |

See [Section 5d](#5d-create-clearpathrobotyaml) for the full file content to paste.

#### `~/clearpath/maps/my_map.{yaml,pgm}`

SLAM-generated occupancy map of the simulation environment. Must be created by the user with a mapping tool (e.g., `slam_toolbox` or `cartographer`) in the same Gazebo world. The `.yaml` metadata file references the `.pgm` image. See [Section 5e](#5e-prepare-a-map-required-for-nav2-localization) for instructions.

#### `/opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml` (patched)

This system file is owned by the `ros-humble-clearpath-nav2-demos` apt package and cannot be shipped in this repo. It must be patched manually (see [Section 5g](#5g-apply-the-system-nav2-patch)).

**The patch, as a diff:**

```diff
--- nav2.yaml.orig
+++ nav2.yaml.patched
@@ general_goal_checker block @@
-      yaw_goal_tolerance: 0.3
+      yaw_goal_tolerance: 0.5

@@ DWB critics block @@
-      RotateToGoal.scale: 32.0
+      RotateToGoal.scale: 16.0
```

**Why these changes were necessary:**

The unpatched J100 configuration caused the robot to oscillate (rock left-right) when approaching a goal orientation:

- `yaw_goal_tolerance: 0.3` rad (~17°) was too tight. The robot's angular inertia and update latency meant it repeatedly overshot and corrected. Widening to `0.5` rad (~29°) allows the controller to declare success before the oscillation loop begins.
- `RotateToGoal.scale: 32.0` made the DWB `RotateToGoal` critic very aggressive — it dominated the cost function and commanded large angular corrections even for small heading errors at the goal. Halving it to `16.0` produces smoother, more damped rotation.

---

## Running

### 7a. Full bringup

```bash
source ~/clearpath_ws/install/setup.bash
ros2 launch j100_nav2_bringup bringup.launch.py
```

The stack comes up in stages over ~30 seconds (see the timeline in [Section 6](#srcj100_nav2_bringuplaunchbringuplandlaunchpy)). Wait until you see the Qt "J100 Nav2 Goal Sender" window appear before sending goals.

To load a different Gazebo world:

```bash
ros2 launch j100_nav2_bringup bringup.launch.py world:=<world_name>
```

### 7b. Sending a navigation goal

**Via the Qt panel (recommended):**

1. Click "Send to Saved Point" to navigate to the pre-configured warehouse destination (x=1.8797, y=-12.0739).
2. Or fill in the X / Y / Yaw spinboxes and click "Send Custom Goal".
3. The Status box updates in real time: PENDING → ACTIVE — N.NN m left → SUCCEEDED.

**Via CLI:**

```bash
ros2 action send_goal /j100_0001/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.88, y: -12.07, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.7179, w: 0.6961}}}}"
```

### 7c. Manual teleoperation (joystick UI)

Run independently from any terminal where the workspace is sourced:

```bash
python3 ~/clearpath_ws/joystick_ui.py
```

Click and drag the joystick knob to drive the robot. Use the sliders to set speed limits. Press **Space** or click the red button to engage the emergency stop. The status bar shows subscriber count and publish rate; a yellow warning appears if no twist_mux subscriber is detected.

---

## Architecture Notes

**Namespace:** Every ROS entity (topics, actions, TF frames, parameter namespaces) lives under `/j100_0001/`. This is set in `robot.yaml` and propagated by the Clearpath launch infrastructure. The `joystick_ui.py` and `nav_goal_ui.py` hard-code this namespace.

**TimerAction sequencing rationale:** Gazebo Fortress takes approximately 8-12 seconds to finish loading the world and spawning the robot before it begins publishing `/clock`. AMCL and Nav2 both require a running clock (they use sim time) and a working TF tree before they can initialize. Launching them immediately causes silent failures. The 10s / 15s / 25s / 30s delays are conservative margins that work reliably on a modern desktop.

**Automatic initial pose:** AMCL requires an initial pose estimate before it can localize. Without one the costmaps remain empty and Nav2 rejects all goals. The launch file publishes a `geometry_msgs/PoseWithCovarianceStamped` at t=25s that places the robot at the map origin with a moderate uncertainty covariance, eliminating the need to click "2D Pose Estimate" in RViz.

**Twist mux priorities:** The Clearpath platform uses `twist_mux` to arbitrate between multiple velocity sources. The joystick UI (publishing to `/j100_0001/cmd_vel`) is registered at priority 1 (highest). Nav2 sends goals via the action interface; the controller publishes at a lower priority. An active Nav2 goal will be preempted if the joystick publishes a non-zero twist.

**Why the nav2.yaml patch lives outside this repo:** The patched file belongs to the `ros-humble-clearpath-nav2-demos` apt package. Shipping a modified copy in this repo would require overriding the package's install path at build time, adding significant complexity. The current approach is simpler but has one drawback: `apt upgrade` will restore the original values. Re-apply the `sed` commands in [Section 5g](#5g-apply-the-system-nav2-patch) after any upgrade of that package.

---

## Troubleshooting

**Robot doesn't move after "Send Goal"**

AMCL may not have received its initial pose yet (if the bringup is still in the first 25 seconds), or may have failed to localize. Check:

```bash
ros2 topic echo /j100_0001/amcl_pose --once
```

If this hangs, AMCL is not publishing. Confirm the map files exist at `~/clearpath/maps/my_map.yaml` and that the localization launch started successfully.

**Robot oscillates at the goal (rocks left-right)**

The nav2.yaml patch was not applied or was overwritten by an apt upgrade. Re-run the `sed` commands from [Section 5g](#5g-apply-the-system-nav2-patch) and restart the bringup.

**`ros2 topic echo /j100_0001/cmd_vel` hangs or shows no data**

QoS mismatch. The joystick UI publishes with `BEST_EFFORT` reliability. Use:

```bash
ros2 topic echo /j100_0001/cmd_vel --qos-reliability best_effort
```

**"Send to Saved Point" sends the robot to a strange location**

The saved coordinates (x=1.8797, y=-12.0739, qz=-0.7179, qw=0.6961) are specific to the Clearpath warehouse Gazebo world. For a different world or map, edit `SAVED_X`, `SAVED_Y`, `SAVED_QZ`, `SAVED_QW` at the top of `src/j100_nav2_bringup/j100_nav2_bringup/nav_goal_ui.py`.

**Screencast files show as text pointers instead of videos**

Git LFS objects were not fetched. Run:

```bash
git lfs pull
```

**`colcon build` fails on clearpath packages**

Ensure `vcs import` completed successfully and that all apt prerequisites from [Section 4](#prerequisites) are installed. Run `rosdep install --from-paths src --ignore-src -r -y` again if packages were added after the first build.

---

## Credits and License

This repository builds on top of packages developed and maintained by [Clearpath Robotics](https://clearpathrobotics.com/):

- [`clearpath_gz`](https://github.com/clearpathrobotics/clearpath_gz) — Gazebo Fortress simulation launch and worlds
- [`clearpath_viz`](https://github.com/clearpathrobotics/clearpath_viz) — RViz navigation visualization launch
- [`clearpath_nav2_demos`](https://github.com/clearpathrobotics/clearpath_nav2_demos) — Nav2 localization and navigation launch files and configuration
- [`clearpath_common`](https://github.com/clearpathrobotics/clearpath_common) — Platform URDF, robot description, and controller configuration
- [`clearpath_config`](https://github.com/clearpathrobotics/clearpath_config) — Robot configuration schema and parsing
- [`clearpath_msgs`](https://github.com/clearpathrobotics/clearpath_msgs) — Clearpath-specific ROS 2 message definitions

**License:** MIT
