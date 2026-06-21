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
- Step 3: UR20 visualization and state control pending

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

## Next Steps

- Build and validate the Step 3 visualization flow using the `ur_description` submodule.
