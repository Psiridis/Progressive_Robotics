# Development Log

## 2026-06-19

- Set up the Step 1 Docker environment on Ubuntu 22.04.
- Installed ROS 2 Humble desktop, Eigen, yaml-cpp, and rviz_visual_tools.
- Added a docker-compose service that builds the image and mounts `./src` into `/ros2_ws/src`.
- Configured the container to source ROS 2 automatically on shell startup.
- Verified the image builds successfully with `docker build`.
- Verified the compose file with `docker compose config`.
