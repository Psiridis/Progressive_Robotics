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
- Step 2c: Client implementation pending
- Step 2d: Server implementation pending
- Step 2e: Condition variable and threading behavior pending
- Step 3: UR20 visualization and state control pending

## Repository Layout

- `Dockerfile`: Builds the ROS 2 Humble development container.
- `docker-compose.yml`: Starts the development container and mounts local source code into `/ros2_ws/src`.
- `src/`: Workspace source directory for ROS 2 packages.
- `src/linear_algebra_service`: Service interface package for Step 2.

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
	- `geometry_msgs/Vector3 b` for vector b
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
[linear_algebra_server] Received request with 3 row(s).
```

**Terminal 2: Start the client**

```bash
ros2 run linear_algebra_nodes client_node
```

Expected output:
```
[linear_algebra_client] Client initialized successfully.
[linear_algebra_client] Sending dummy request...
[linear_algebra_client] Server response: success=true, message='Request processed successfully.'
```

Both nodes should exit cleanly after the request/response exchange completes.

## Next Steps

- Complete Step 2c with YAML configuration loading in the client.
- Complete Step 2d with full least squares computation in the server.
- Complete Step 2e with subscriber, condition variable, and thread synchronization.
- Continue to Step 3 after Step 2 integration is validated.
