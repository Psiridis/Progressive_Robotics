# Development Log

## 2026-06-19

- Set up the Step 1 Docker environment on Ubuntu 22.04.
- Installed ROS 2 Humble desktop, Eigen, yaml-cpp, and rviz_visual_tools.
- Added a docker-compose service that builds the image and mounts `./src` into `/ros2_ws/src`.
- Configured the container to source ROS 2 automatically on shell startup.
- Verified the image builds successfully with `docker build`.
- Verified the compose file with `docker compose config`.

## 2026-06-20

- Renamed service package to `linear_algebra_service` for assignment naming consistency.
- Added Step 2a service interface in `src/linear_algebra_service/srv/LeastSquareContract.srv`.
- Updated interface package metadata and build files:
	- `src/linear_algebra_service/package.xml`
	- `src/linear_algebra_service/CMakeLists.txt`
- Confirmed no leftover `least_square_service` references remain in `src`.

Planned Step 2 milestones:

- Step 2a: Service interface package
- Step 2b: Dummy client and dummy server nodes
- Step 2c: Full client implementation (YAML loading, inverse transform, publish)
- Step 2d: Full server implementation (least squares, random transform, response)
- Step 2e: Subscriber, condition variable, and wait thread behavior
