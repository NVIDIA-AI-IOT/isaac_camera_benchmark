# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import pprint

from isaac_world.isaac_world import IsaacWorld

DEFAULT_CONFIG = {
    # Path of the scenario to launch relative to the nucleus server base path. Scenario must contain a carter robot.
    # If the scene contains animated humans, the script expects to find them under /World/Humans.
    # Example scenarios are /Isaac/Samples/NvBlox/carter_warehouse_navigation_with_people.usd
    # or /Isaac/Samples/NvBlox/carter_warehouse_navigation.usd
    # or /Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd
    # 'scenario_path': '/Isaac/Samples/NvBlox/carter_warehouse_navigation.usd',
    'scenario_path': 'omniverse://localhost/Users/rbonghi/carter_warehouse_navigation.usd',
    # Directory location to save the waypoints in a yaml file
    'anim_people_waypoint_dir': '',
    # Path to the world to create a navigation mesh.
    'environment_prim_path': '/World/WareHouse',
    # Choose whether we generate random waypoint or run sim
    'random_command_generation': False,
    # Number of waypoints to generate for each human in the scene.
    'num_waypoints': 5,
    # Run the simulation headless.
    'headless': False,
    # If used, animation extensions for humans will be enabled.
    'with_people': False,
    # Choose whether to use generated/custom command file or to use the default one to run the people animation
    'use_generated_command_file': False,
    # The rate (in hz) that we step the simulation at.
    'tick_rate_hz': 20,
    # Enable odometry carter
    'enable_odometry': True,
    # Number camera to test
    # Camera resolutions:
    # [640, 480]
    # [1024, 768]
    # [1280, 720]
    # HD [1920, 1080]
    # 4K [3840, 2160]
    'camera': {
        'left': {
            'resolution': [1280, 720],
            'enable_rgb': True,
            'enable_depth': True,
        },
        'right': {
            'resolution': [1280, 720],
            'enable_rgb': True,
            'enable_depth': True,
        },
        'extra': {
            'resolution': [640, 480],
            'enable_rgb': False,
            'enable_depth': False,
        },
        'extra2': {
            'resolution': [640, 480],
            'enable_rgb': False,
            'enable_depth': False,
        },
    }
}

ADDITIONAL_EXTENSIONS_BASE = [
    'omni.isaac.ros2_bridge-humble', 'omni.isaac.sensor']

ADDITIONAL_EXTENSIONS_PEOPLE = [
    'omni.anim.people', 'omni.anim.navigation.bundle', 'omni.anim.timeline',
    'omni.anim.graph.bundle', 'omni.anim.graph.core', 'omni.anim.graph.ui',
    'omni.anim.retarget.bundle', 'omni.anim.retarget.core',
    'omni.anim.retarget.ui', 'omni.kit.scripting']


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


def enable_extensions_for_sim(with_people: bool = False):
    """
    Enable the required extensions for the simulation.

    Args:
        with_people (bool, optional): Loads the human animation extensions if the scene with
                                      humans is requested. Defaults to False.

    """
    from omni.isaac.core.utils.extensions import enable_extension
    add_exten = ADDITIONAL_EXTENSIONS_BASE
    if with_people:
        add_exten = ADDITIONAL_EXTENSIONS_BASE + ADDITIONAL_EXTENSIONS_PEOPLE
    for idx in range(len(add_exten)):
        enable_extension(add_exten[idx])


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Sample app for running Carter in a Warehouse')
    parser.add_argument('--config_path',
                        default='config.json',
                        help='Path to the world to create a navigation mesh.')
    # This allows for IsaacSim options to be passed on the SimulationApp.
    args, unknown = parser.parse_known_args()

    config = read_config(args.config_path)

    # If we want to generate the command file then we run the simulation headless
    if config['random_command_generation']:
        config['headless'] = True

    # Check if the command file directory is given if it has to be generated or used for the
    # simulation
    if config['random_command_generation'] or config['use_generated_command_file']:
        if not config['anim_people_waypoint_dir']:
            raise ValueError(
                'Input to command file directory required if custom command file has to be \
                    generated/used!!'
            )

    # Indented format with 2 spaces for indentation
    pprint.pprint(config, indent=2)

    headless = config['headless']
    with_people = config['with_people']

    # Start up the simulator
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({
        'renderer': 'RayTracedLighting',
        'headless': headless
    })

    # Enables the simulation extensions
    enable_extensions_for_sim(with_people)

    import rclpy
    from isaac_world.topic_reader import TopicHzReader
    rclpy.init()
    # Isaac SIM world
    isaac_world = IsaacWorld(config, simulation_app)

    # Start simulation
    isaac_world.start_simulation()
    # Initialize node
    topic_hz_reader = TopicHzReader()
    # topic_hz_reader = None
    # Run simulation
    isaac_world.run_simulation(topic_hz_reader)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_hz_reader.destroy_node()
    rclpy.shutdown()
# EOF
