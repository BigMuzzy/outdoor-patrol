# VSCode ROS2 Workspace Template

This template will get you set up using ROS2 with VSCode as your IDE.

See [how I develop with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace.

## Features

### Style

ROS2-approved formatters are included in the IDE.  

* **c++** uncrustify; config from `ament_uncrustify`
* **python** autopep8; vscode settings consistent with the [style guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)

### Tasks

There are many pre-defined tasks, see [`.vscode/tasks.json`](.vscode/tasks.json) for a complete listing.  Feel free to adjust them to suit your needs.  

Take a look at [how I develop using tasks](https://www.allisonthackston.com/articles/vscode_tasks.html) for an idea on how I use tasks in my development.

### Debugging

This template sets up debugging for python files, gdb for cpp programs and ROS launch files.  See [`.vscode/launch.json`](.vscode/launch.json) for configuration details.

### Continuous Integration

The template also comes with basic continuous integration set up. See [`.github/workflows/ros.yaml`](/.github/workflows/ros.yaml).

To remove a linter just delete it's name from this line:

```yaml
      matrix:
          linter: [cppcheck, cpplint, uncrustify, lint_cmake, xmllint, flake8, pep257]
```

## Quickstart — M0 (teleop baseline)

This workspace's first milestone is **M0**: drive the bare diff-drive
chassis from a keyboard over USB-CDC, with the ESP32-S3 micro-ROS
controller as the `cmd_vel` sink. See
[`src/robot-research/notes/outdoor-patrol/recipe/implementation-plan.md`](src/robot-research/notes/outdoor-patrol/recipe/implementation-plan.md#m0--teleop-baseline-diff-drive-cmd_vel)
for the full milestone spec and acceptance tests.

### Prerequisites for M0

- ESP32-S3 firmware from
  [`src/esp32-s3-uros-controller`](src/esp32-s3-uros-controller) flashed
  and connected over USB-C (`/dev/ttyACM0` by default).
- **Physical kill switch** wired to motor power. Required from M0
  onward; software has no E-stop in this milestone.
- `micro_ros_agent` built from source. There is no jazzy debian, so the
  upstream meta-build tool
  [`micro_ros_setup`](https://github.com/micro-ROS/micro_ros_setup) is
  vendored as a submodule at
  [`src/micro_ros_setup`](src/micro_ros_setup) and bootstrapped by
  [`scripts/setup-uros-agent.sh`](scripts/setup-uros-agent.sh) (see
  *Build and run* below). The agent and its pinned XRCE-DDS / vendor
  dependencies are built into a sibling workspace at `uros_agent_ws/`
  (gitignored) rather than the main `install/` tree, because
  `micro_ros_setup` is a meta-build tool that fetches and patches a
  second set of sources.

### Build and run

```bash
git clone --recurse-submodules <this-repo>
./setup.sh        # vcs import + rosdep install
./build.sh        # colcon build --merge-install --symlink-install
source install/setup.bash

# One-time (or whenever the micro_ros_setup submodule moves):
./scripts/setup-uros-agent.sh
source uros_agent_ws/install/local_setup.bash

# Terminal 1: agent + robot_state_publisher
ros2 launch outdoor_patrol_bringup teleop.launch.py serial_dev:=/dev/ttyACM0

# Terminal 2: keyboard teleop (needs a real TTY, not auto-launched)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3 (optional): RViz with the M0 preset
ros2 launch outdoor_patrol_bringup rviz.launch.py
```

`config/chassis.yaml` is the single source of truth for wheel radius,
separation, and speed clamps; mirror any change into the firmware
constants in
[`firmware/main/diff_drive.h`](src/esp32-s3-uros-controller/firmware/main/diff_drive.h).

### M1 — wheel odometry + TF tree

M1 adds the `odom → base_link` transform via a single-input
`robot_localization` EKF
([`outdoor_patrol_loc`](src/outdoor_patrol_loc)) that consumes the
chassis `/odom`. The firmware publishes the odom *message* only; the EKF
is the sole owner of the transform (REP-105 single-writer).

```bash
# Terminal 1: agent + robot_state_publisher + EKF (odom -> base_link)
ros2 launch outdoor_patrol_bringup odometry.launch.py serial_dev:=/dev/ttyACM0

# Terminal 2: keyboard teleop (needs a real TTY, not auto-launched)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3 (optional): RViz with the M1 odometry preset (fixed frame = odom)
ros2 launch outdoor_patrol_bringup rviz.launch.py \
  rviz_config:=$(ros2 pkg prefix outdoor_patrol_bringup)/share/outdoor_patrol_bringup/config/odometry.rviz
```

Acceptance: joystick-drive a taped 2 m × 2 m square, plot
`/odometry/filtered` in RViz (closure < 0.3 m), confirm
`tf2_echo odom base_link` is smooth, and `ros2 topic hz /odom` ≥ 20 Hz.

### GNSS global localization (interim, ADR-012)

Brings up globally-referenced localization (`map → odom → base_link`) by
fusing wheel odometry with RTK GNSS, using the UM982's **dual-antenna heading
in place of an IMU**
([ADR-012](src/robot-research/notes/outdoor-patrol/recipe/decisions.md)). This
pulls GNSS ahead of the IMU (M2) and LIO (M4); when those land, `map → odom`
ownership moves to LIO. **Localization only** — it does not authorize
autonomous motion (the M3 safety brake gates that).

One launch starts the whole stack: micro-ROS agent + `robot_state_publisher`,
the UM982 driver + NTRIP (RTK), the dual-EKF + `navsat_transform` +
`confidence_gate`, and RViz (fixed frame = `map`).

```bash
# Terminal 1: full stack. Point ntrip_params_file at your real caster creds.
ros2 launch outdoor_patrol_bringup gnss_localization.launch.py \
  serial_dev:=/dev/ttyACM0 \
  ntrip_params_file:=$(pwd)/ntrip.yaml

# Terminal 2: keyboard teleop (needs a real TTY, not auto-launched)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Before a real run, set the heading `yaw_offset` in
[`heading_to_imu`](src/outdoor_patrol_loc/src/heading_to_imu.cpp) from the
measured antenna-baseline mount angle — until then the `map` orientation is
unaligned. The datum is auto-set on the first RTK fix, so **start near the
dock** to keep coordinates small.

Acceptance: at the dock origin, drive a 20 m line and confirm the EKF pose
tracks GNSS within 0.3 m; cover the antenna mid-drive and confirm the gate
inflates covariance (no `map` jump) and recovers cleanly.

---

## How to use this template

### Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

#### NVidia support

To make nvidia driver and opengl available in docker, follow the installation instructions for docker-nvidia.
They include the steps in docker and add the additional gpu layer.

* [docker-nvidia (includes docker install and additional installation for NVidia GPU accelerated hosts)](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

### Get the template

Click on "use this template"

![template_use](https://user-images.githubusercontent.com/6098197/91331899-43f23b80-e780-11ea-92c8-b4665ce126f1.png)

### Create your repository

On the next dialog, name the repository you would like to start and decide if you want all of the branches, or the default branch.

> [!IMPORTANT]
> 
> The new default branch supports any version of ROS by setting the appropriate version you want in the 'FROM' line in `.devcontainer/Dockerfile`
>
> By default, this is set to `osrf/ros:jazzy-desktop-full`

![template_new](https://user-images.githubusercontent.com/6098197/91332035-713ee980-e780-11ea-81d3-13b170f568b0.png)

Github will then create a new repository with the contents of this one in your account.  It grabs the latest changes as "initial commit".

### Clone your repo

Now you can clone your repo as normal

![template_download](https://user-images.githubusercontent.com/6098197/91332342-e4e0f680-e780-11ea-9525-49b0afa0e4bb.png)

### Open it in vscode

Now that you've cloned your repo onto your computer, you can open it in VSCode (File->Open Folder).

When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

### Update the template with your code

1. Specify the repositories you want to include in your workspace in `src/ros2.repos` or delete `src/ros2.repos` and develop directly within the workspace.
2. If you are using a `ros2.repos` file, import the contents `Terminal->Run Task..->import from workspace file`
3. Install dependencies `Terminal->Run Task..->install dependencies`
4. (optional) Adjust scripts to your liking.  These scripts are used both within tasks and CI.
   * `setup.sh` The setup commands for your code.  Default to import workspace and install dependencies.
   * `build.sh` The build commands for your code.  Default to `--merge-install` and `--symlink-install`
   * `test.sh` The test commands for your code.
5. Develop!

## FAQ

### XAuthority

If you see the error:

```text
Authorization required, but no authorization protocol specified Unable to open display: :0 Authorization required, but no authorization protocol specified
```

You may need to update the UID/GID to match yours.  In `.devcontainer/devcontainer.json` update the lines that are marked `Change to match your UID` and `Change to match your GID`

.devcontainer/devcontainer.json

```jsonc
 "build": {
  "args": {
   ...
   // "USERNAME": "ros",
   // "USER_UID": "1000", //Change to match your UID
   // "USER_GID": "1000" // Change to match your GID
  },
 },
 ...
 "runArgs": [
  ...
  "--volume=/run/user/1000:/run/user/1000", // Change 1000 to match your UID
  ...
 ],
```

### XDisplay

If you see the error:

```text
Couldn't open X display in GLXGLSupport::getGLDisplay at ./.obj-x86_64-linux-gnu/ogre_vendor-prefix/src/ogre_vendor/RenderSystems/GLSupport/src/GLX/OgreGLXGLSupport.cpp
```

You need to remove or comment out the wayland options

```jsonc
 "runArgs": [
  ...
  // Wayland host
  //"--volume=/mnt/wslg:/mnt/wslg",
  // "--volume=/run/user/1000:/run/user/1000",
  // uncomment to use intel iGPU
  // "--device=/dev/dri"
  ...
 ],
 ...
  "containerEnv": {
  ...
  // For Wayland
  // "WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}",
  // "XDG_RUNTIME_DIR": "${localEnv:XDG_RUNTIME_DIR}",
  // "QT_QPA_PLATFORM": "wayland", // Force Wayland
  ...
 },
```

### WSL2

#### The gui doesn't show up

This is likely because the DISPLAY environment variable is not getting set properly.

1. Find out what your DISPLAY variable should be

      In your WSL2 Ubuntu instance

      ```bash
      echo $DISPLAY
      ```

2. Copy that value into the `.devcontainer/devcontainer.json` file

      ```jsonc
      "containerEnv": {
        "DISPLAY": ":0",
      }
      ```

#### I want to use vGPU

If you want to access the vGPU through WSL2, you'll need to add additional components to the `.devcontainer/devcontainer.json` file in accordance to [these directions](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md)

```jsonc
 "runArgs": [
  "--network=host",
  "--cap-add=SYS_PTRACE",
  "--security-opt=seccomp:unconfined",
  "--security-opt=apparmor:unconfined",
  "--volume=/tmp/.X11-unix:/tmp/.X11-unix",
  "--volume=/mnt/wslg:/mnt/wslg",
  "--volume=/usr/lib/wsl:/usr/lib/wsl",
  "--device=/dev/dxg",
  "--gpus=all"
 ],
 "containerEnv": {
  "DISPLAY": "${localEnv:DISPLAY}", // Needed for GUI try ":0" for windows
  "WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}",
  "XDG_RUNTIME_DIR": "${localEnv:XDG_RUNTIME_DIR}",
  "PULSE_SERVER": "${localEnv:PULSE_SERVER}",
  "LD_LIBRARY_PATH": "/usr/lib/wsl/lib",
  "LIBGL_ALWAYS_SOFTWARE": "1" // Needed for software rendering of opengl
 },
```

### Repos are not showing up in VS Code source control

This is likely because vscode doesn't necessarily know about other repositories unless you've added them directly.

```text
File->Add Folder To Workspace
```

![Screenshot-26](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/d8711320-2c16-463b-9d67-5bd9314acc7f)

Or you've added them as a git submodule.

![Screenshot-27](https://github.com/athackst/vscode_ros2_workspace/assets/6098197/8ebc9aac-9d70-4b53-aa52-9b5b108dc935)

To add all of the repos in your *.repos file, run the script

```bash
python3 .devcontainer/repos_to_submodules.py
```

or run the task titled `add submodules from .repos`

### Error handling for GPU acceleration

#### Docker image cannot be built:

The dockerfile can be built but using devcontainer.json results in error messages like "docker container cannot connect to device [[gpu]]" means docker itself is installed, but not the above mentioned nvidia part.

Solution is, to follow the guide and the test with nvidia-smi as indicated here:

- [docker-nvidia(for GPU acceleration on Nvidia GPU hosts)](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

#### Programs in Docker cannot access GPU

Error messages that show lacking GPU acceleration (in docker terminal)

```bash
sudo apt-get update   && sudo apt-get install -y -qq glmark2   && glmark2
```

results in:

```bash
   libGL error: No matching fbConfigs or visuals found
   libGL error: failed to load driver: swrast
      X Error of failed request:  GLXBadContext
   Major opcode of failed request:  151 (GLX)
   Minor opcode of failed request:  6 (X_GLXIsDirect)
   Serial number of failed request:  48
   Current serial number in output stream:  47
```

Solution is, to follow the guide and the test with nvidia-smi as indicated here: 
[docker-nvidia(for GPU acceleration on Nvidia GPU hosts)](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

#### more information

https://wiki.ros.org/docker/Tutorials/GUI
https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker
