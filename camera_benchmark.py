# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
from omni.isaac.kit import SimulationApp
import sys

import argparse

parser = argparse.ArgumentParser(description="Ros2 Bridge Sample")
parser.add_argument('--config_path',
                    default='config.json',
                    help='Path to the world to create a navigation mesh.')
args, unknown = parser.parse_known_args()

ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

DEFAULT_CONFIG = {
    'simulation': {"renderer": "RayTracedLighting", "headless": False},
    'camera': [
        {'translate': [-1, 5, 1], 'resolution': [640, 480]},
        {'translate': [-1, 1, 6], 'resolution': [640, 480]},
        {'translate': [-1, 7, 3], 'resolution': [640, 480]},
        # {'translate': [1, 2, 3], 'resolution': [640, 480]},
    ]
}

def read_config(filename):
    try:
        with open(filename, 'r') as file:
            config_data = json.load(file)
    except FileNotFoundError:
        print(
            f"Config file '{filename}' not found. Using default configuration.")
        return DEFAULT_CONFIG

    # Update default config with values from the file
    config = DEFAULT_CONFIG.copy()
    config.update(config_data)
    return config

# Load config file
config = read_config(args.config_path)
simulation_app = SimulationApp(config['simulation'])


import json
import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage, extensions, nucleus
from pxr import Gf, UsdGeom, Usd
from omni.kit.viewport.utility import get_active_viewport
import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def create_camera(translate=[-1, 5, 1], resolution=[640, 480], number_camera=0):
    camera_stage_path = "/Camera" + f"{number_camera}"
    ros_camera_graph_path = ROS_CAMERA_GRAPH_PATH + f"{number_camera}"
    # Creating a Camera prim
    camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    xform_api.SetTranslate(Gf.Vec3d(translate[0], translate[1], translate[2]))
    xform_api.SetRotate((90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim.GetHorizontalApertureAttr().Set(21)
    camera_prim.GetVerticalApertureAttr().Set(16)
    camera_prim.GetProjectionAttr().Set("perspective")
    camera_prim.GetFocalLengthAttr().Set(24)
    camera_prim.GetFocusDistanceAttr().Set(400)

    simulation_app.update()
    
    viewport_name = f"Viewport{number_camera}" if number_camera != 0 else ""

    # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": ros_camera_graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                ("setViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:execOut", "setViewportResolution.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("createViewport.outputs:viewport", "setViewportResolution.inputs:viewport"),
                ("setViewportResolution.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:name", viewport_name),
                ("createViewport.inputs:viewportId", number_camera),
                ("setViewportResolution.inputs:width", resolution[0]),
                ("setViewportResolution.inputs:height", resolution[1]),
                ("setCamera.inputs:cameraPrim", f"{camera_stage_path}"),
                ("cameraHelperRgb.inputs:frameId", "sim_camera"),
                ("cameraHelperRgb.inputs:topicName", f"/Camera{number_camera}/rgb"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperInfo.inputs:frameId", "sim_camera"),
                ("cameraHelperInfo.inputs:topicName", f"/Camera{number_camera}/camera_info"),
                ("cameraHelperInfo.inputs:type", "camera_info"),
                ("cameraHelperDepth.inputs:frameId", "sim_camera"),
                ("cameraHelperDepth.inputs:topicName", f"/Camera{number_camera}/depth"),
                ("cameraHelperDepth.inputs:type", "depth"),
            ],
        },
    )

    # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
    og.Controller.evaluate_sync(ros_camera_graph)

    simulation_app.update()
    
    return xform_api


class BenchmarkCamera(Node):
    def __init__(self, config):
        super().__init__("benchmark_camera_node")
        # Run ROS2 node in a separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        # Init variables
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.window_size = 10000  # window_size
        self.times = []
        self.fps = 0
        # Get camera list from config file
        self.xform_api_camera = []

        self.simulation_context = SimulationContext(stage_units_in_meters=1.0)

        # Locate Isaac Sim assets folder to load environment and robot stages
        assets_root_path = nucleus.get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()

        # Loading the simple_room environment
        stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

        if 'camera' not in config or len(config['camera']) == 0:
            carb.log_error("There are no camera in list, please add one in config file")
            simulation_app.close()
            sys.exit()

        for idx, camera in enumerate(config['camera']):
            self.xform_api_camera += [create_camera(translate=camera['translate'], resolution=camera['resolution'], number_camera=idx)]

        self.subscription = self.create_subscription(
            Image,  # Replace with the actual message type you're subscribing to
            '/Camera0/rgb',  # Replace with the actual topic name
            self.callback_hz,
            qos_profile_sensor_data,
        )

    def callback_hz(self, msg):

        curr_rostime = self.get_clock().now()

        if curr_rostime.nanoseconds == 0:
            if len(self.times) > 0:
                print('time has reset, resetting counters')
                self.times = []
            return
        
        curr = curr_rostime.nanoseconds
        msg_t0 = self.msg_t0
        if msg_t0 < 0 or msg_t0 > curr:
            self.msg_t0 = curr
            self.msg_tn = curr
            self.times = []
        else:
            self.times.append(curr - self.msg_tn)
            self.msg_tn = curr

        if len(self.times) > self.window_size:
            self.times.pop(0)

    def plot_benchmark(self, fps):
            if not self.times:
                return
            elif self.last_printed_tn == 0:
                self.last_printed_tn = self.msg_tn
                return
            elif self.msg_tn < self.last_printed_tn + 1e9:
                return
            # Get frequency every one minute
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1. / mean if mean > 0. else 0
            self.last_printed_tn = self.msg_tn
            # Print benchmark
            rate_print = rate * 1e9
            self.get_logger().info(
                f"ROS avg: {rate_print:.3f} Hz - Isaac SIM FPs: {fps:.2f}")

    def run_simulation(self):
        # Need to initialize physics getting any articulation..etc
        self.simulation_context.initialize_physics()
        self.simulation_context.play()
        frame = 0
        # Dock all viewports
        n_vieports = len(self.xform_api_camera)
        if n_vieports > 1:
            viewport = omni.ui.Workspace.get_window('Viewport')
            for idx in reversed(range(1, n_vieports)):
                viewport_idx = omni.ui.Workspace.get_window(f"Viewport{idx}")
                viewport_idx.dock_in(viewport, omni.ui.DockPosition.RIGHT)
        # Run simulation
        while simulation_app.is_running():
            # Run with a fixed step size
            self.simulation_context.step(render=True)
            # rclpy.spin_once(self, timeout_sec=0.0)
            if self.simulation_context.is_playing():
                if self.simulation_context.current_time_step_index == 0:
                    self.simulation_context.reset()
            # Get viewport fps and plot benchmark
            viewport_api = get_active_viewport()
            self.plot_benchmark(viewport_api.fps)
            # Rotate camera by 0.5 degree every frame
            for xform_api in self.xform_api_camera:
                xform_api.SetRotate((90, 0, frame / 4.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            frame = frame + 1
        # Cleanup
        self.simulation_context.stop()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    # Start simulation
    subscriber = BenchmarkCamera(config)
    subscriber.run_simulation()
    # Cleanup
    rclpy.shutdown()
# EOF
