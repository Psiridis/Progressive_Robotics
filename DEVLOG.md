# Development Log

## 2026-06-19

- Set up the Step 1 Docker environment on Ubuntu 22.04.
- Installed ROS 2 Humble desktop, Eigen, yaml-cpp, and rviz_visual_tools.
- Added a docker-compose service that builds the image and mounts `./src` into `/ros2_ws/src`.
- Configured the container to source ROS 2 automatically on shell startup.
- Verified the image builds successfully with `docker build`.
- Verified the compose file with `docker compose config`.

## 2026-06-20

### Step 2a: Service Interface (completed)

- Renamed service package to `linear_algebra_service` for assignment naming consistency.
- Added Step 2a service interface in `src/linear_algebra_service/srv/LeastSquareContract.srv`.
- Updated interface package metadata and build files:
	- `src/linear_algebra_service/package.xml`
	- `src/linear_algebra_service/CMakeLists.txt`
- Confirmed no leftover `least_square_service` references remain in `src`.

### Step 2b: Dummy Client and Server Nodes (completed)

- Created `linear_algebra_nodes` executable package with two nodes.
- Implemented `ServerNode` class in `server_node.cpp`:
  - Constructor accepts node name, init_service method accepts service endpoint name
  - Service callback logs request row count, returns dummy success response
  - Proper error handling with non-zero exit codes on startup failure
- Implemented `ClientNode` class in `client_node.cpp`:
  - Constructor accepts node name, init_client method accepts service endpoint name
  - Waits for service availability with 1-second timeout intervals
  - Sends one dummy request with 3×3 identity matrix and zero vector
  - Logs server response and shuts down cleanly
- Both nodes use matching service endpoint name `least_square_service`
- Applied consistent C++ formatting via .clang-format (2-space indent, 100-column limit, function braces on new line)
- Updated `README.md` with Step 2b build and test instructions
- Ready to commit after running integration test

### Step 2c: Full Client Input Path (completed)

- Refactored `ClientNode` request flow into smaller helper methods:
  - `load_file(...)`
  - `validate_yaml_schema(...)`
  - `parse_yaml_file(...)`
  - `dispatch_request(...)`
- Replaced hard-coded dummy request values with YAML-driven input loading.
- Added schema validation before parsing:
  - `a_rows` must exist and be a non-empty sequence
  - every `a_rows[i]` must contain exactly 3 values
  - `b` must exist and contain exactly 3 values
- Added clear error logging for file load, schema validation, and parsing failures.
- Updated client runtime usage to require config path argument:
  - `ros2 run linear_algebra_nodes client_node /ros2_ws/src/linear_algebra_nodes/config/input.yaml`
- Added centralized client logging helper with consistent `[CLIENT]` prefix formatting.
- Verified request/response flow with YAML input and server dummy response.

Remaining Step 2 milestones:

- Step 2d: Full server implementation (least squares, random transform, response)
- Step 2e: Subscriber, condition variable, and wait thread behavior
