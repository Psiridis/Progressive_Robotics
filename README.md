# Progressive Robotics Technical Assessment

This repository contains my implementation of the Progressive Robotics technical assessment.

Progress is organized in milestones:

- Step 1: Dockerized ROS 2 Humble environment
- Step 2a: Service interface package
- Step 2b-2e: Nodes and runtime behavior
- Step 3: UR20 visualization and state control

## Current Status

- Step 1: Docker environment completed
- Step 2a: Service interface package completed
- Step 2b: Dummy client and dummy server completed
- Step 2c: Client implementation completed
- Step 2d: Server implementation completed
- Step 2e: Condition variable and threading behavior completed
- Step 3: UR description dependency added
- Step 3: UR20 model + gripper integration completed
- Step 3: ur20_display node implementation completed
- Step 3: Headless and optional RViz launch flow completed

## Repository Layout

- `Dockerfile`: Builds the ROS 2 Humble development container.
- `docker-compose.yml`: Starts the development container and mounts local source code into `/ros2_ws/src`.
- `src/`: Workspace source directory for ROS 2 packages.
- `src/linear_algebra_service`: Service interface package for Step 2.
- `src/linear_algebra_nodes`: Client/server package for Step 2 runtime behavior.
- `src/ur_description`: Git submodule that tracks the Universal Robots ROS 2 description package used for Step 3.

## Clone With Submodules

Clone the repository with its external robot-description dependency:

```bash
git clone --recurse-submodules <repo-url>
```

If the repository is already cloned, initialize and fetch the submodule with:

```bash
git submodule update --init --recursive
```

## Step 1: Build and Run

### Build the container

```bash
docker compose build
```

### Start the container

```bash
docker compose up
```

The container starts with ROS 2 sourced automatically through `/root/.bashrc`, so ROS 2 commands are available after entering the shell.

### Open a shell inside the running container

```bash
docker compose exec ros2 bash
```

## Installed Tooling

The container installs the following packages required by the assignment:

- `ros-humble-desktop`
- `ros-dev-tools`
- `libeigen3-dev`
- `libyaml-cpp-dev`
- `ros-humble-rviz-visual-tools`

## Notes

- The workspace source directory on the host is mounted to `/ros2_ws/src` in the container.
- I verified that the Docker image builds successfully and that the compose file parses correctly.

## Step 2a: Service Interface Package

The package `linear_algebra_service` defines the custom service used in Step 2.

- Service file: `src/linear_algebra_service/srv/LeastSquareContract.srv`
- Request:
	- `geometry_msgs/Point[] a_rows` for matrix A rows
	- `float64[] b` for M-dimensional vector b
- Response:
	- `geometry_msgs/Vector3 x_prime`
	- `geometry_msgs/Quaternion r_prime`
	- `geometry_msgs/Vector3 d_prime`
	- `bool success`
	- `string message`

### Build Step 2a package

Inside the container:

```bash
cd /ros2_ws
colcon build --packages-select linear_algebra_service
source install/setup.bash
```

### Verify generated interface

Inside the container after sourcing:

```bash
ros2 interface show linear_algebra_service/srv/LeastSquareContract
```

## Step 2b: Dummy Client and Server Nodes

The package `linear_algebra_nodes` contains two executable nodes that communicate via the `LeastSquareContract` service.

- Server node: `src/linear_algebra_nodes/src/server_node.cpp`
  - Advertises the service on endpoint `least_square_service`
  - Logs incoming requests
  - Returns a dummy response with `success=true`
- Client node: `src/linear_algebra_nodes/src/client_node.cpp`
  - Connects to service on endpoint `least_square_service`
  - Sends one dummy request with a 3×3 identity matrix and zero vector
  - Logs the server response
  - Shuts down cleanly after receiving response

### Build Step 2b packages

Inside the container:

```bash
cd /ros2_ws
colcon build --packages-select linear_algebra_service linear_algebra_nodes
source install/setup.bash
```

### Test the client and server

**Terminal 1: Start the server**

```bash
ros2 run linear_algebra_nodes server_node
```

Expected output:
```
[linear_algebra_server] Service initialized successfully.
[linear_algebra_server] Received request with 4 row(s).
```

**Terminal 2: Start the client**

```bash
ros2 run linear_algebra_nodes client_node /ros2_ws/src/linear_algebra_nodes/config/input.yaml
```

Expected output:
```
[linear_algebra_client] Client initialized successfully.
[linear_algebra_client] Sending request with 4 row(s) from '/ros2_ws/src/linear_algebra_nodes/config/input.yaml'.
[linear_algebra_client] Server response: success=true, message='Request processed successfully.'
[linear_algebra_client] Recovered x: [..., ..., ...]
[linear_algebra_client] Published x: [..., ..., ...] to topic.
```

Both nodes should exit cleanly after the request/response exchange completes.

## Step 2c: Client YAML Input and Validation

The client now loads request data from a YAML file instead of using hard-coded dummy values.

- Input file: `src/linear_algebra_nodes/config/input.yaml`
- Runtime usage:
	- `ros2 run linear_algebra_nodes client_node /ros2_ws/src/linear_algebra_nodes/config/input.yaml`

### Supported YAML schema

```yaml
a_rows:
	- [1.0, 0.0, 0.0]
	- [0.0, 1.0, 0.0]
	- [0.0, 0.0, 1.0]
	- [1.0, 1.0, 1.0]

b: [0.0, 0.0, 0.0, 0.0]
```

### Validation rules

- `a_rows` must exist and be a sequence
- `a_rows` must contain at least 3 rows
- each row in `a_rows` must have exactly 3 values
- `b` must exist and be a sequence
- `b` must have the same number of values as `a_rows`

If validation fails, the client logs an error and exits without sending a request.

## Step 2d: Server Least-Squares and Client Reverse Transform

The server now performs the full least-squares and random-transform pipeline.

- Builds matrix `A` and vector `b` from request data
- Solves least-squares system using Eigen
- Generates random transform `(R', d')`
- Computes transformed solution `x' = R' * x + d'`
- Returns `x_prime`, `r_prime`, `d_prime`, `success`, and `message`

The client now performs reverse transform after receiving a successful response.

- Reads `x_prime`, `r_prime`, and `d_prime`
- Computes recovered solution `x = R'^(-1) * (x' - d')`
- Logs recovered `x`

### Step 2d runtime notes

- Server logs incoming `A` rows and `b` values for traceability.
- Server random generator is stored as a member variable for better performance under repeated requests.
- Random rotation axis sampling uses normalized Gaussian components for isotropic direction sampling.

## Step 2e: Subscriber, Condition Variable, and Wait Thread

Step 2e is implemented in the `linear_algebra_nodes` package.

- Client publishes recovered `x` to topic `least_square_topic` after successful response inversion.
- Server subscribes to `least_square_topic`.
- Server stores the received message and notifies a condition variable.
- A separate server thread waits on that condition variable.
- When notified, the waiting thread prints the received message and waits again for future messages.

### Step 2e runtime notes

- The subscriber callback and waiting thread share data protected by a mutex.
- The condition variable avoids busy waiting and wakes the thread only when a new message arrives.

## Step 3: UR Description Dependency

Step 3 now includes the external Universal Robots description repository as a git submodule.

- Submodule path: `src/ur_description`
- Remote repository: `https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git`
- Tracked branch: `humble`

This keeps the vendor robot description assets versioned separately while making them available inside the ROS 2 workspace for UR20 visualization work.

## Step 3: UR20 Visualization and State Control

The package `ur20_display` now contains the full Step 3 implementation.

- UR20 + custom gripper model: `src/ur20_display/urdf/ur20_with_gripper.urdf.xacro`
- Launch file: `src/ur20_display/launch/ur20_display_launch.py`
- State/TF node: `src/ur20_display/src/ur20_display_node.cpp`
- Node declaration header: `src/ur20_display/include/ur20_display/ur20_display_node.hpp`
- RViz config: `src/ur20_display/rviz/ur20_display.rviz`

### Node behavior (`ur20_display_node`)

- Publishes `sensor_msgs/msg/JointState` on `/joint_states` using parameterized joint values.
- Listens to TF transforms:
	- `T_elbow_gripper`
	- `T_world_elbow`
	- `T_world_gripper`
- Converts transforms to `Eigen::Isometry3d`.
- Verifies chain consistency:

$$
T_{world,gripper} = T_{world,elbow} \cdot T_{elbow,gripper}
$$

- Publishes visualization markers on `/ur20_display_markers`:
	- oriented XYZ frame (line-list axes)
	- text label `Tf_elbow_gripper`

### Launch arguments

- `use_sim_time` (default `false`)
  - Use simulation clock instead of wall clock

- `enable_rviz` (default `false`)
  - Launch RViz window for visualization

- `use_software_rendering` (default `true`)
  - Force Mesa software (CPU) rendering for RViz
  - Essential when running through Docker/XQuartz on macOS
  - Set to `false` only on native Linux with full GPU/OpenGL support

`enable_rviz:=false` keeps runtime headless for clean development logs.

### Build

Inside the container:

```bash
cd /ros2_ws
colcon build --packages-select ur20_display
source install/setup.bash
```

### Run (headless)

```bash
ros2 launch ur20_display ur20_display_launch.py enable_rviz:=false
```

### Run (with RViz)

```bash
ros2 launch ur20_display ur20_display_launch.py enable_rviz:=true
```

### macOS GUI Forwarding Setup (Required for RViz)

RViz on macOS requires X11 forwarding through Docker. Follow these steps:

#### Step 1: Install XQuartz (one-time)

```bash
brew install --cask xquartz
```

#### Step 2: Configure XQuartz Security Settings (one-time)

1. Start XQuartz:
   ```bash
   open -a XQuartz
   ```

2. Go to XQuartz menu > Preferences

3. Navigate to the **Security** tab

4. **Enable** "Allow connections from network clients"

5. Restart XQuartz (close and reopen)

#### Step 3: Allow Localhost X11 Connections (each terminal session)

Before launching the container, run:

```bash
xhost +localhost
```

Expected output:
```
localhost being added to access control list
```

#### Step 4: Launch with RViz

Now run the launch command with `enable_rviz:=true`:

```bash
ros2 launch ur20_display ur20_display_launch.py enable_rviz:=true
```

RViz should open within 3-5 seconds.

#### Troubleshooting macOS RViz Issues

**RViz window doesn't appear:**

1. Verify XQuartz is running:
   ```bash
   ps aux | grep XQuartz
   ```
   If not running, open it:
   ```bash
   open -a XQuartz
   ```

2. Verify xhost permissions:
   ```bash
   xhost
   ```
   Should show `localhost` in the access list. If missing, run:
   ```bash
   xhost +localhost
   ```

3. Test X11 forwarding directly (inside container):
   ```bash
   ros2 run rviz2 rviz2
   ```
   If this also shows no window, the issue is X11 forwarding, not ROS.

4. Check docker-compose.yml DISPLAY variable:
   ```bash
   grep DISPLAY docker-compose.yml
   ```
   Should show `DISPLAY=host.docker.internal:0`

5. Verify software rendering is enabled in launch file:
   ```bash
   ros2 launch ur20_display ur20_display_launch.py enable_rviz:=true use_software_rendering:=true
   ```

**RViz opens but runs slowly or appears glitchy:**

- This is normal on macOS with Docker/XQuartz forwarding
- RViz is using software (CPU) rendering instead of GPU
- To reduce latency, close unnecessary markers or simplify the scene

**Qt error: "Could not connect to display":**

1. Ensure XQuartz is running
2. Verify xhost has localhost enabled
3. Check that DISPLAY environment variable is set:
   ```bash
   echo $DISPLAY
   ```
   Should show something like `host.docker.internal:0`

### Validation checklist

In a second shell, after launch:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

1. Check topics:

```bash
ros2 topic list
```

Expected topics include `/joint_states`, `/tf`, `/tf_static`, `/ur20_display_markers`.

2. Check one `JointState` sample:

```bash
ros2 topic echo /joint_states --once
```

3. Check one MarkerArray sample:

```bash
ros2 topic echo /ur20_display_markers visualization_msgs/msg/MarkerArray --once
```

Expected marker payload includes:
- line-list marker for axes
- text marker with `text: Tf_elbow_gripper`

## Next Steps

- Optional: add bonus trajectory animation publisher.
- Optional: add Python trajectory plotter node.
