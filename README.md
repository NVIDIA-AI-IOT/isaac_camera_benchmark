# isaac_camera_benchmark

This tool run a simple test to check the performance of your desktop on Isaac SIM.
You can run multiple test, with and without ROS and read:

* Camera performance from 1 to 4 cameras
* Change resolution
* ROS2

## Hardware required

Workstation:

1. x86/64 machine
2. Install Ubuntu 20.04
3. NVIDIA Graphic card with RTX
4. Display
5. Keyboard and Mouse

#### Run demo

Clone this repository and move to repository folder

```console
git clone https://github.com/nvidia_iot/isaac_camera_benchmark.git
cd isaac_camera_benchmark
```

Run the installer

```console
./run_camera_benchmark.sh.sh
```

#### NVIDIA Isaac SIM

Follow the documentation on NVIDIA Isaac SIM [Workstation install](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)

1. Download the [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. [Install Omniverse Launcher](https://docs.omniverse.nvidia.com/prod_launcher/prod_launcher/installing_launcher.html)
3. Install [Cache](https://docs.omniverse.nvidia.com/prod_nucleus/prod_utilities/cache/installation/workstation.html) from the Omniverse Launcher
4. Install [Nucleus](https://docs.omniverse.nvidia.com/prod_nucleus/prod_nucleus/workstation/installation.html) from the Omniverse Launcher

Open Omniverse Launcher

![Omniverse launcher](https://docs.omniverse.nvidia.com/app_isaacsim/_images/isaac_main_launcher_exchange.png)

Move to Library and choice "Omniverse Isaac SIM" and download the latest 2022.2 version

![Omniverse library](https://docs.omniverse.nvidia.com/app_isaacsim/_images/isaac_main_launcher_library.png)