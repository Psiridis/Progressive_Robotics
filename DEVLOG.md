# Development Log

## 2026-06-21

### Step 3: UR Description Dependency

- Added `src/ur_description` as a git submodule.
- Configured the submodule in `.gitmodules` to track `https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git` on the `humble` branch.
- Committed the submodule pointer so the workspace now records the exact upstream revision used for UR20 visualization work.
- Updated `README.md` to document the new dependency and the required submodule initialization workflow.

### Step 3: UR20 model + launch scaffold (completed)

- Created `ur20_display` package scaffold and integrated UR20 model through `xacro`.
- Added custom fixed-joint gripper link in `ur20_with_gripper.urdf.xacro`.
- Added launch orchestration for:
  - `robot_state_publisher`
  - `ur20_display_node`
  - conditional `rviz2` startup
- Added RViz configuration for grid/robot model/default view.
- Added launch argument `enable_rviz` so development can run headless.

### Step 3: ur20_display_node implementation (completed)

- Refactored node into declaration/implementation split:
  - `include/ur20_display/ur20_display_node.hpp`
  - `src/ur20_display_node.cpp`
- Implemented joint-state publisher from parameterized joint configuration.
- Implemented TF lookup pipeline using small helper methods.
- Parameterized frame names (`world_frame`, `elbow_frame`, `gripper_frame`) and defaulted elbow frame to `forearm_link`.
- Implemented Eigen transform conversion and chain verification:
  - `T_world_gripper = T_world_elbow * T_elbow_gripper`
- Implemented marker publication of:
  - oriented frame axes (LINE_LIST)
  - text marker `Tf_elbow_gripper`
- Added runtime guard for invalid parameter sizes (`joint_names` vs `joint_positions`).

### Step 3: Build + runtime verification (completed)

- Built `ur20_display` package successfully in Docker.
- Verified headless launch path:
  - `ros2 launch ur20_display ur20_display_launch.py enable_rviz:=false`
- Confirmed robot_state_publisher loads all expected UR20 + gripper segments.
- Confirmed `/joint_states` is published with the configured six UR joint names.
- Confirmed `/ur20_display_markers` publishes `visualization_msgs/msg/MarkerArray` including:
  - oriented frame marker
  - label marker with text `Tf_elbow_gripper`

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

### Step 2d: Server Least-Squares and Transform Pipeline (completed)

- Upgraded server request handling to full Step 2d behavior.
- Improved random transform generation quality and runtime behavior:
  - Moved RNG engine from function-local creation to a `ServerNode` member variable.
  - Note: reusing a member RNG avoids reseeding and repeated engine construction per request, which improves performance under sustained service calls.
  - Switched axis sampling to normal-distribution components with normalization for better isotropic direction sampling.
  - Replaced `M_PI` usage with a portable local `constexpr` PI constant.
  - Future note: optional deterministic seeding (for reproducible tests) can be added later if needed.
- Implemented least-squares solution on server using Eigen.
- Implemented transformed response generation:
  - `x_prime = R' * x + d'`
  - Response fields: `x_prime`, `r_prime`, `d_prime`, `success`, `message`
- Added descriptive server logging of received `A` rows and `b` values.
- Implemented client-side reverse transform in response handler:
  - Recover `x` with `x = R'^(-1) * (x' - d')`
  - Log recovered `x` for verification.

### Step 2e: Subscriber, Condition Variable, and Wait Thread (completed)

- Implemented client publisher on topic `least_square_topic`.
- Fixed client publish behavior to match assignment:
  - Publish recovered `x` (not request `b`) after successful inverse transformation.
- Implemented server subscriber for `geometry_msgs/msg/Vector3` on `least_square_topic`.
- Added condition-variable synchronization path in server:
  - Subscriber callback stores message and notifies condition variable.
  - Dedicated wait thread blocks until notified.
  - When notified, wait thread prints received message and then waits again.
- Added server thread lifecycle management (start on node creation, clean join on destruction).

Remaining Step 2 milestones:

- Step 2 complete
