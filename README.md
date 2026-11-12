# Autonomous Valet Parking with Hybrid A* Path Planning and Pure-Pursuit Control on ROS 2 

A simulated car enters a parking lot, is assigned a free bay, plans a drivable path (including reversing into the bay), and follows it. When a shopping cart suddenly appears in the aisle, the car detects it with its lidar, replans, and parks.

Built with Python, NumPy, Matplotlib, ROS 2 Jazzy and RViz2. Simulation only: no real vehicle is used.

<!-- Add a screenshot or GIF here, e.g. ![demo](docs/demo.gif) -->

## Features

- **Hybrid A\* planner** with a kinematic bicycle model: forward and reverse motion, minimum turning radius, collision checking
- **Pure-pursuit controller** with direction switching (forward to reverse) and rule-based emergency braking
- **Obstacle detection** by comparing the lidar scan with the map; new obstacles trigger replanning
- **ROS 2 system** of 6 nodes (map, vehicle simulator, planner, controller, mission, recorder) plus an RViz2 view
- **Standalone mode** that runs the same core code without ROS, for fast experiments
- **Automatic outputs**: MP4 video and PNG graphs saved after every run
- **Modular code**: all tunable numbers are in one `config.py`; 25 automated tests

## How it works

```
 core/  (plain Python, no ROS)                    nodes/  (ROS 2, connects core/ to topics)
 ------------------------------                   -----------------------------------------
 lot.py            map, bays, collisions, lidar    map_node          publishes /map
 bicycle.py        vehicle model                   vehicle_sim_node  simulated car + lidar
 lidar.py          scan + unknown-obstacle test    planner_node      goal -> /planned_path, replanning
 hybrid_astar.py   path planner                    controller_node   /planned_path -> /cmd_vel
 path_follower.py  controller + emergency brake    mission_node      sends the parking goal
 obstacles.py      known map + replan rules        recorder_node     saves video and images
 mission.py        full mission without ROS
 render.py         images and video
```

Topics: `/map`, `/odom`, `/scan`, `/goal_pose`, `/planned_path`, `/cmd_vel`, `/controller_status`.

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy (on Windows: WSL2 + Ubuntu 24.04; RViz2 works through WSLg)
- `python3-numpy`, `python3-matplotlib`, `python3-pytest`, `ffmpeg` (without `ffmpeg` you get a GIF instead of an MP4)

### Install (Ubuntu 24.04)

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-numpy python3-matplotlib python3-pytest ffmpeg
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Official guide if a step fails: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## Build

```bash
git clone https://github.com/<your-username>/av-valet-hybrid-astar-ros2.git
cd av-valet-hybrid-astar-ros2/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

With `--symlink-install`, edits to Python files take effect without rebuilding. Rebuild only after adding a node or changing `setup.py`, `package.xml` or the launch file.

## Run

Open an Ubuntu terminal and run `source install/setup.bash` from `ros2_ws` in every new terminal.

### Option 1: one terminal (recommended)

```bash
ros2 launch av_valet valet.launch.py
```

RViz2 opens. After 4 s the car is assigned bay 6 and plans a path. An orange cart appears in the aisle, the car replans, reverses into the bay, and the recorder saves the video and images. Then everything shuts down by itself.

Launch options:

```bash
ros2 launch av_valet valet.launch.py bay:=16 reverse_in:=false cart:=false   # free bays: 3, 6, 12, 16
ros2 launch av_valet valet.launch.py rviz:=false                              # no RViz window
ros2 launch av_valet valet.launch.py record:=false                            # do not save files
```

You can also click **2D Goal Pose** in RViz2 and drag in the lot to send your own goal (the drag direction is the heading of the rear axle).

### Option 2: node by node (6 terminals)

Run these in order, one per terminal. Start `mission_node` last. Add a 7th terminal with `ros2 run av_valet recorder_node` (started before the mission node) if you want the video.

```bash
ros2 run av_valet map_node
ros2 run av_valet vehicle_sim_node
ros2 run av_valet planner_node
ros2 run av_valet controller_node
ros2 run rviz2 rviz2 -d ~/ros2_ws/install/av_valet/share/av_valet/rviz/valet.rviz
ros2 run av_valet mission_node --ros-args -p bay:=6 -p reverse_in:=true
```

(Adjust the RViz path to wherever your `ros2_ws` is.) Stop each node with Ctrl+C.

### Option 3: without ROS (fastest)

```bash
cd ros2_ws/src/av_valet
python3 -m av_valet.standalone --test                  # 25 self-tests, about 15 s
python3 -m av_valet.standalone                         # run, print a report, save video + images
python3 -m av_valet.standalone --bay 16 --forward-in --no-cart
python3 -m av_valet.standalone --show                  # also open the animation window
```

## Outputs

Saved to `~/robot_sim_outputs/av_valet/` (change `OUT_DIR` in `config.py` or pass `--out FOLDER` to the standalone run):

| File | Content |
|---|---|
| `ros2_valet_parking.mp4` | animation of the run (ROS mode) |
| `ros2_route_overview.png`, `ros2_final_frame.png`, `ros2_speed_steering.png` | route, final pose, speed and steering graphs |
| `valet_parking.mp4` and matching PNGs | the same for the standalone run |

## Inspect a running system

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /controller_status      # idle | tracking | switching | blocked | arrived
```

## Project layout

```
ros2_ws/src/av_valet/
  av_valet/
    config.py          every tunable number (lot, car, lidar, planner, controller, thresholds, topics, output folder)
    standalone.py      command line for the no-ROS run
    selftest.py        the 25 tests
    core/              algorithms, no ROS
    nodes/             ROS 2 nodes
  launch/valet.launch.py
  rviz/valet.rviz
  setup.py, package.xml, test/
```

To change the cart position, free bays, car size, speeds or replan thresholds, edit `config.py`. After any change run `python3 -m av_valet.standalone --test`.

## Tests

```bash
cd ros2_ws && colcon test --packages-select av_valet && colcon test-result --verbose
```

## Results and known limitations

The headline scenario (bay 6, reverse-in, cart appears in the aisle) parks inside the bay lines with no collisions. Other combinations are not all clean yet:

- Bay 12 nose-in, and bays 3, 6 and 16 nose-in without a cart, park cleanly.
- Reversing into bays 3, 12 and 16 with the cart can brush a neighbouring car.
- Nose-in parking with the cart can end 11-17 degrees off the bay axis.

Other limits: the vehicle pose is assumed perfectly known (no localisation), the unknown-obstacle test compares the lidar with the map (a real vehicle could not rely on that for emergency braking), there is a single static obstacle, and `/cmd_vel` carries a steering angle as a simplification. This is a simulation and says nothing about real-world safety.

## Troubleshooting

| Problem | Fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash` |
| `Package 'av_valet' not found` | run `colcon build --symlink-install` and `source install/setup.bash` |
| `package 'rviz2' not found` | `sudo apt install ros-jazzy-rviz2`, or run with `rviz:=false` |
| No RViz window on Windows | `wsl --update`, then `wsl --shutdown` and reopen Ubuntu |
| GIF instead of MP4 | `sudo apt install ffmpeg` |

## License

Add your license here (for example MIT).
