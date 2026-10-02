# Intrinsic Tactile Sensing Docker

This folder provides a Docker-based development environment for the `its_ros2` workspace.

Files added
- `Dockerfile` - image for ROS2 (Humble) development with build tools and dependencies.
- `docker/entrypoint.sh` - container entrypoint that sources ROS and the overlay workspace.
- `docker-compose.yaml` - compose file to build and run a development container mounting the `ros2/` workspace.

Quick start

1. From the project root, export your UID/GID so the image can create a matching user (optional but recommended):

```bash
export UID=$(id -u)
export GID=$(id -g)
```

2. Build the image with docker compose (this will pick up `UID`/`GID` if set):

```bash
docker compose build --build-arg UID=${UID:-1000} --build-arg GID=${GID:-1000}
```

3. Allow X connections (if you plan to run GUI tools) and start the container:

```bash
xhost +local:root
docker compose up -d
```

4. Open a shell inside the container:

```bash
docker compose exec its_dev bash
```

Inside the container
- The `ros2` workspace is mounted at `/home/dev/ros2` (adjust if you built with a different user).
- Source ROS 2 (already done by the entrypoint) and build your workspace:

```bash
# inside container
cd ~/ros2
colcon build --packages-select its_ros2
source install/setup.bash
ros2 run its_ros2 check_package_node
```

Notes
- The compose file uses `network_mode: host` for ROS2 discovery. This works on Linux only.
- If your host user has a different UID/GID, rebuild with `--build-arg UID=$(id -u) --build-arg GID=$(id -g)`.
- The container is based on `ros:humble-ros-core`. Change the base image in `Dockerfile` if you need a different ROS distribution.

Want changes?
If you want a VNC-based GUI, persistent Docker volumes, or additional system libraries added, tell me what you need and I can update the Dockerfile and compose file.
