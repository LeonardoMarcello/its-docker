# Intrinsic Tactile Sensing Docker

This folder provides a Docker-based development environment for the `its_ros2` workspace,
and a minimal production image that runs ITS out of the box (see [Production](#production)).


1. Build the devel image with docker compose:

```bash
docker compose build
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
- The `ros2` workspace is mounted at `/home/its_dev/ros2` (adjust if you built with a different user).
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

---

## Production

The production image (`Dockerfile.prod`, compose service `its`) contains the compiled ITS
workspace and nothing else: no compilers, no headers, no development tools. It is meant to be
started on the robot PC and left running.

### What is inside the image

| Item                                                              | Location in the container             |
| ----------------------------------------------------------------- | ------------------------------------- |
| ITS workspace (`its_msgs`, `its_ros2`), Release build, stripped   | `/opt/its_ws/install`                 |
| coal (collision library) shared libraries, v3.0.4                 | `/usr/local/lib`                      |
| Runtime libraries detected from the binaries (`ldd`) + CycloneDDS | system / `/opt/ros/humble`            |
| Configs and meshes (`its_ros2/config`, `its_ros2/assets`)         | `/opt/its_ws/install/share/its_ros2/` |
| Non-root user                                                     | `its` (UID/GID 1000 by default)       |

Executables shipped: `its_node`, `soft_its_node`, `wrench_compensator` and `soft_its_viz` (always
included, with the OpenGL/Mesa runtime). The test node `check_package_node` is not shipped.

The image is about 630 MB (vs. ~1.9 GB for the development image).

### Prerequisites

- Linux host with Docker Engine and the Docker Compose v2 plugin (`docker compose version`).
- The host must reach the F/T sensors' ROS 2 network: the container uses `network_mode: host`.
- The F/T driver must publish `geometry_msgs/WrenchStamped` on `/<sensor.id>/netft_data`
  using the same `ROS_DOMAIN_ID` and a compatible RMW (see [Middleware](#middleware-rmw-and-domain)).

### 1. Build the image

From the repository root:

```bash
docker compose build its
```

This builds `Dockerfile.prod` and tags the result as `its:latest`. The first build compiles coal
and takes several minutes; later builds reuse the cache and only recompile the workspace.

The source code is **copied into the image at build time**: after any change to C++ code, messages,
launch files or CMake, rebuild the image. Config changes do not need a rebuild (see step 3).

Optional build arguments (use plain `docker build`, compose does not pass them):

| Argument   | Default  | Effect                       |
| ---------- | -------- | ---------------------------- |
| `COAL_REF` | `v3.0.4` | coal git tag/branch to build |


```bash
# example: matching the host user
docker build -f Dockerfile.prod \
    --build-arg COAL_REF="v3.0.4" \
    -t its:latest .
```

### 2. Run

```bash
# start in the background (runs: ros2 launch its_ros2 its_bringup.launch.py)
docker compose up
```

On start you should see one `its_node` per sensor block of `ahand_bringup.yaml` and, if
`compensate_fingertips: true`, the `wrench_compensator`:

```
[INFO] [its_node-1]: process started with pid [27]
...
[INFO] [wrench_compensator-5]: process started with pid [35]
[its_node-2] [INFO] [...] [sensor_2]: Solver: Closed-Form
```

To run a different launch file, override the command (`run --rm` gives a one-off container):

```bash
# single fingertip, standard ITS, using config/setup.yaml
docker compose --profile prod run --rm its \
    ros2 launch its_ros2 its.launch.py algorithm:=standard setup:=setup.yaml

# soft ITS
docker compose --profile prod run --rm its \
    ros2 launch its_ros2 its.launch.py algorithm:=soft setup:=setup.yaml
```

> `its.launch.py` must be given `setup:=...` explicitly, and it always starts `soft_its_viz`
> (requires the display set up as in step 5).

Interactive shell in the running container:

```bash
docker compose --profile prod exec its bash
```

### 3. Configuration

`./ros2/src/its_ros2/config` on the host is mounted over the installed config folder
(`/opt/its_ws/install/share/its_ros2/config`). Edit the files on the host and restart:

```bash
docker compose --profile prod restart its
```

Files used by `its_bringup.launch.py`:

- `ahand_bringup.yaml`
  - `compensate_fingertips`: start the `wrench_compensator` (`true`/`false`).
  - one block per sensor (`sensor_1`, `sensor_2`, ...): `sensor.id`, `fingertip.*`
    (surface, displacement, orientation, mesh). Each block starts one `its_node` named after it.
  - `soft_its`: rate, `wrench_msgs`, `algorithm.force_threshold` and `algorithm.method.name`:
    `"Levenberg-Marquardt"` | `"Gauss-Newton"` | `"Closed-Form"` | `"Wrench-Method"` | `"Custom"`.
  - `soft_viz` (commented out by default): when present, a `soft_its_viz` window is started per sensor.
- `bias_correction.yaml`: parameters of the `wrench_compensator`.

Meshes referenced by the configs must be inside the image (`its_ros2/assets`, rebuild to add new
ones) or mounted into the container at the configured path.

### 4. Topics

For each sensor block (`<sensor.id>`, `<fingertip.id>` from the config):

| Direction | Topic                                                         | Type                                                     |
| --------- | ------------------------------------------------------------- | -------------------------------------------------------- |
| in        | `/<sensor.id>/netft_data`                                     | `geometry_msgs/WrenchStamped` (force [N], torque [N mm]) |
| in        | `its_<fingertip.id>/initial_guess`                            | `its_msgs/SoftContactSensingProblemSolutionStamped`      |
| out       | `soft_csp_<fingertip.id>/solution`                            | `its_msgs/SoftContactSensingProblemSolutionStamped`      |
| out       | `its_<fingertip.id>/wrench` (if `soft_its.wrench_msgs: true`) | `geometry_msgs/WrenchStamped` in the contact frame       |
| out       | TF `<fingertip.id>` → `<fingertip.id>_PoC`                    | contact frame, z along the surface normal                |

`wrench_compensator` subscribes `/<id>/netft_data` and publishes `/<id>_compensated/netft_data`
for every id listed in `bias_correction.yaml`; this is why the sensor ids in `ahand_bringup.yaml`
end with `_compensated`.

Check from the host (or from `docker compose --profile prod exec its bash`):

```bash
ros2 topic list -t | grep -E "soft_csp|its_|netft"
ros2 topic echo soft_csp_<fingertip.id>/solution --once
```

### 5. Visualizer (optional)

`soft_its_viz` is always included in the image; it is started when the config has a `soft_viz` block.
The compose service already forwards `DISPLAY` and `/tmp/.X11-unix`; allow the container to
open windows on the host X server before starting it:

```bash
xhost +local:
docker compose --profile prod up -d its
```

### Middleware (RMW) and domain

The compose service sets, overridable from the host environment or a `.env` file:

| Variable             | Default              | Notes                                             |
| -------------------- | -------------------- | ------------------------------------------------- |
| `ROS_DOMAIN_ID`      | `0`                  | must match the F/T driver and any consumer        |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | `rmw_fastrtps_cpp` is also available in the image |

```bash
ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_fastrtps_cpp docker compose --profile prod up -d its
```

### Troubleshooting

| Symptom                                       | Cause / fix                                                                                                                                                                       |
| --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Nodes start but no solution is published      | No data on `/<sensor.id>/netft_data`: check `ROS_DOMAIN_ID`, RMW, and that the sensor ids in the config match the driver (`_compensated` ids need `compensate_fingertips: true`). |
| Solution messages are all zeros               | Force below `soft_its.algorithm.force_threshold`: the node publishes an empty solution in that case.                                                                              |
| `launch configuration 'setup' does not exist` | `its.launch.py` called without `setup:=<file>.yaml`.                                                                                                                              |
| `cannot open display` / GLUT error            | Run `xhost +local:` on the host and check that `DISPLAY` is set where you run `docker compose`.                                                                                   |
| Config edits have no effect                   | Restart the container; check the mount with `docker compose --profile prod exec its ls /opt/its_ws/install/share/its_ros2/config`.                                                |
| Code changes have no effect                   | The code is baked into the image: `docker compose --profile prod build its` and `up -d` again.                                                                                    |

Want changes?
If you want a VNC-based GUI, persistent Docker volumes, or additional system libraries added, tell me what you need and I can update the Dockerfile and compose file.
