# ROS 2 Humble Development on Mac M2 (Apple Silicon)

This project establishes a robust ROS 2 development environment using VS Code Dev Containers. It solves the common graphical issues on macOS by offloading visualization to **Foxglove Studio** via a WebSocket bridge.

## 1. Prerequisites

* **Docker Desktop:** [Download here](https://www.docker.com/products/docker-desktop/).
* *Note:* Ensure it is running and the whale icon is steady in your menu bar.


* **VS Code:** [Download here](https://code.visualstudio.com/).
* *Extension:* Install the "Dev Containers" extension (ID: `ms-vscode-remote.remote-containers`).


* **Foxglove Studio:** [Download here](https://foxglove.dev/download) (Desktop App recommended).

---

## 2. Project Structure

Create a new folder for your workspace (e.g., `ros2_ws`) and set up the following structure:

```text
ros2_ws/
├── .devcontainer/
│   ├── Dockerfile
│   └── devcontainer.json
└── src/
    └── (Your ROS packages will go here)

```

---

## 3. Configuration Files

Create the two files inside the `.devcontainer` folder with the content below.

### `.devcontainer/Dockerfile`

This builds the environment, installs ROS 2 Humble, developer tools, and the crucial bridge for visualization.

```dockerfile
FROM osrf/ros:humble-desktop-full

# 1. Install basic dev tools and the Foxglove Bridge
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    vim \
    nano \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# 2. Initialize rosdep
RUN rosdep update

# 3. Auto-source the workspace in every new terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

```

### `.devcontainer/devcontainer.json`

This configures VS Code to run inside the container, manages the network, and installs useful extensions automatically.

```json
{
    "name": "ROS 2 Humble M2",
    "build": {
        "dockerfile": "Dockerfile",
        "context": ".."
    },
    "privileged": true,
    "remoteUser": "root",

    // Expose the port for Foxglove Studio
    "forwardPorts": [8765],

    // Install recommended VS Code extensions
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "ms-python.python",
                "ms-iot.vscode-ros",
                "twxs.cmake",
                "yzhang.markdown-all-in-one"
            ]
        }
    },

    // Use host networking to simplify ROS communication
    "runArgs": [
        "--network=host"
    ],

    "postCreateCommand": "rosdep update && echo 'Environment Ready!'"
}

```

---

## 4. How to Start

1. **Open in VS Code:** Open the `ros2_ws` folder in VS Code.
2. **Launch Container:**
* A popup should appear: *"Folder contains a Dev Container configuration..."* -> Click **Reopen in Container**.
* *Manual method:* Press `Cmd+Shift+P` -> Type **"Dev Containers: Reopen in Container"**.


3. **Wait:** The first run will take a few minutes to download and build the image.

---

## 5. Usage Workflow

### A. Start the Visualization Bridge

To see your robot/data on your Mac, you must run the bridge. Open a terminal inside VS Code and run:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

```

*You should see: `Server listening on port 8765`.*

### B. Connect Visualization (On Mac)

1. Open **Foxglove Studio** on your Mac.
2. Click **Open Connection** -> **Foxglove WebSocket**.
3. URL: `ws://localhost:8765` -> Click **Open**.

### C. Run Code / Simulation

Open a **new terminal** (split pane) in VS Code to run your actual ROS nodes.
Example (Testing 3D visualization):

```bash
ros2 launch dummy_robot_bringup dummy_robot_bringup.launch.py

```

*Go back to Foxglove, add a **3D Panel**, and you will see the robot moving.*

---

## 6. Developing Your Own Package

Here is the quick cheat-sheet for creating and building code in this environment.

**1. Create a Package:**

```bash
cd /workspaces/ros2_ws/src
ros2 pkg create my_package --build-type ament_python --dependencies rclpy

```

**2. Write Code:**
Edit files in `src/my_package/my_package/`. (Don't forget to update `setup.py` entry points!).

**3. Build:**

```bash
cd /workspaces/ros2_ws
colcon build --packages-select my_package

```

**4. Source & Run:**

```bash
source install/setup.bash
ros2 run my_package <node_name>

```

---

## 7. Troubleshooting

* **"Port 8765 already in use":** You might have an old container running. Run `docker stop $(docker ps -q)` on your Mac terminal to kill everything.
* **"WebSocket connection failed":** Ensure the `ros2 launch foxglove_bridge ...` command is actually running in a VS Code terminal.
* **No Data in Foxglove:** Ensure you added the correct panel (e.g., "3D" for robots, "Plot" for numbers) and selected a Topic in the panel settings.