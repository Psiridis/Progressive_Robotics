# Progressive Robotics Technical Assessment

This repository contains my implementation of the Progressive Robotics technical assessment. The current commit focuses on Step 1: a Dockerized ROS 2 Humble development environment on Ubuntu 22.04.

## Current Status

- Step 1: Docker environment completed
- Step 2: Linear algebra ROS 2 service pending
- Step 3: UR20 visualization and state control pending

## Repository Layout

- `Dockerfile`: Builds the ROS 2 Humble development container.
- `docker-compose.yml`: Starts the development container and mounts local source code into `/ros2_ws/src`.
- `src/`: Workspace source directory to be populated with ROS 2 packages for later steps.

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

## Next Steps

- Implement the ROS 2 linear algebra service for Step 2.
- Add the UR20 description package and visualization node for Step 3.
