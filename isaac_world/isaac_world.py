

import omni
import random
import os
import time
import pprint

def rebuild_nav_mesh():
    """
    Rebuild the navmesh with the correct settings. Used for the people to move around.

    Called only when the sim with people is requested.
    """
    # Navigation mesh rebake settings
    import omni.kit.commands
    print('Rebuilding navigation mesh...')

    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.navigation.core/navMesh/config/height',
        value=1.5)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.navigation.core/navMesh/config/radius',
        value=0.5)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.navigation.core/navMesh/config/maxSlope',
        value=60.0)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.navigation.core/navMesh/config/maxClimb',
        value=0.2)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeDelaySeconds',
        value=4)
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/persistent/exts/omni.anim.navigation.core/navMesh/viewNavMesh',
        value=False)
    print('Navigation mesh rebuilt.')


def create_people_commands(environment_prim_path: str,
                           anim_people_command_dir: str,
                           max_number_tries: int,
                           num_waypoints_per_human: int):
    """
    Create the occupancy grid which returns the free points for the humans to move.

    These points are then randomly sampled and assigned to each person and is saved as a txt file
    which can be used as the command file for people animation. It directly gets the context of
    the scene once the scene is opened and played in IsaacSim.

    Args:
        environment_prim_path (str): The warehouse enviroment prim path
        anim_people_command_dir (str): The local directory to be used for storing the command file
        max_number_tries (int): The number of times the script attempts to select a new waypoint
        for a human.
        num_waypoints_per_human (int): The number of waypoints to create per human.

    """
    from omni.isaac.occupancy_map import _occupancy_map
    import omni.anim.navigation.core as navcore
    from omni.isaac.core.prims import XFormPrim
    import carb
    navigation_interface = navcore.acquire_interface()
    print('Creating randomized paths for people...')
    bounding_box = omni.usd.get_context().compute_path_world_bounding_box(
        environment_prim_path)

    physx = omni.physx.acquire_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()

    generator = _occupancy_map.Generator(physx, stage_id)
    # 0.05m cell size, output buffer will have 0 for occupied cells, 1 for
    # unoccupied, and 6 for cells that cannot be seen
    # this assumes your usd stage units are in m, and not cm
    generator.update_settings(.05, 0, 1, 6)
    # Setting the transform for the generator
    generator.set_transform(
        # The starting location to map from
        (0, 0, 0.025),
        # The min bound
        (bounding_box[0][0], bounding_box[0][1], 0.025),
        # The max bound
        (bounding_box[1][0], bounding_box[1][1], 0.025))
    generator.generate2d()
    # Get locations of free points
    free_points = generator.get_free_positions()
    # Find out number of humans in scene and create a list with their names
    human_names = []
    human_prims = []
    for human in omni.usd.get_context().get_stage().GetPrimAtPath(
            '/World/Humans').GetAllChildren():
        human_name = human.GetChildren()[0].GetName()
        # To remove biped setup prim from the list
        if 'CharacterAnimation' in human_name or 'Biped_Setup' in human_name:
            continue
        human_names.append(human_name)
        human_prims.append(XFormPrim(human.GetPath().pathString))

    random_waypoints = {}
    for human_name, human_prim in zip(human_names, human_prims):
        # Get the human initial position.
        start_point, _ = human_prim.get_world_pose()
        # Initialize target waypoints.
        random_waypoints[human_name] = []
        for _ in range(num_waypoints_per_human):
            current_tries = 0
            path = None
            # Sample for a maximum number of tries then give up.
            while path is None and current_tries < max_number_tries:
                new_waypoint = random.sample(free_points, 1)
                # Check that a path exists between the starting position and the destination
                path = navigation_interface.query_navmesh_path(
                    carb.Float3(start_point), new_waypoint[0])
                current_tries += 1
            if path is not None:
                new_waypoint[0].z = 0.0
                random_waypoints[human_name].append(new_waypoint[0])
                print(
                    f'Found path for {human_name} from {start_point} to {new_waypoint[0]} after \
                     {current_tries} tries')
                start_point = new_waypoint[0]
            else:
                print(
                    f'Could not find path for {human_name} after {max_number_tries}, skipping')

    # Save as command file
    command_file_path = os.path.join(
        anim_people_command_dir, 'human_cmd_file.txt')
    print(f'Saving randomized commands to {command_file_path}')
    with open(command_file_path, 'w') as file:
        for human_name, waypoints in random_waypoints.items():
            human_command_line = f'{human_name} GoTo '
            for next_waypoint in waypoints:
                human_command_line += f'{next_waypoint[0]} {next_waypoint[1]} 0 '
            human_command_line += '_\n'
            file.write(human_command_line)


def update_people_command_file_path(anim_people_command_dir: str):
    """
    Update the command file path settings in the simulation scene to the custom one.

    Args:
        anim_people_command_dir (str): The directory where the generated/custom command file is
                                       stored.

    """
    import omni.kit.commands
    omni.kit.commands.execute(
        'ChangeSetting',
        path='/exts/omni.anim.people/command_settings/command_file_path',
        value=anim_people_command_dir + '/human_cmd_file.txt')


def configure_camera(carter_prim_path, controller, name, config_camera):
    # print(f"----------------------------- {name} -------------------------------------")
    # Create camera node and set target resolution
    viewport_node = controller.node(f'isaac_create_viewport_{name}',
                                    f'{carter_prim_path}/ROS_Cameras')
    viewport_node.get_attribute('inputs:name').set(name)
    # Change resolution
    camera_data = config_camera.get(name, {})
    resolution = camera_data.get('resolution', [640, 480])
    # Create camera node and set target resolution
    resolution_node = controller.node(f'isaac_set_viewport_resolution_{name}',
                                      f'{carter_prim_path}/ROS_Cameras')
    resolution_node.get_attribute('inputs:width').set(resolution[0])
    resolution_node.get_attribute('inputs:height').set(resolution[1])
    # Change publication topics
    rgb = controller.node(f'ros2_create_camera_{name}_rgb',
                          f'{carter_prim_path}/ROS_Cameras')
    rgb.get_attribute('inputs:topicName').set(
        f'/front/stereo_camera/{name}/rgb')
    info = controller.node(f'ros2_create_camera_{name}_info',
                           f'{carter_prim_path}/ROS_Cameras')
    info.get_attribute('inputs:topicName').set(
        f'/front/stereo_camera/{name}/camera_info')
    depth = controller.node(f'ros2_create_camera_{name}_depth',
                            f'{carter_prim_path}/ROS_Cameras')
    depth.get_attribute('inputs:topicName').set(
        f'/front/stereo_camera/{name}/depth')

    # Read enable camera
    enable_camera_rgb = camera_data.get('enable_rgb', True)
    enable_camera_depth = camera_data.get('enable_depth', True)
    enable_camera = enable_camera_rgb or enable_camera_depth
    print(name, enable_camera)

    # Enable camera
    enable = controller.node(f'enable_camera_{name}',
                             f'{carter_prim_path}/ROS_Cameras')
    enable.get_attribute('inputs:condition').set(enable_camera)
    # Enable camera rgb
    enable_rgb = controller.node(f'enable_camera_{name}_rgb',
                                 f'{carter_prim_path}/ROS_Cameras')
    enable_rgb.get_attribute('inputs:condition').set(enable_camera_rgb)
    # Enable camera depth
    enable_depth = controller.node(f'enable_camera_{name}_depth',
                                   f'{carter_prim_path}/ROS_Cameras')
    enable_depth.get_attribute('inputs:condition').set(enable_camera_depth)

    return info


class IsaacWorld:

    def __init__(self, config, simulation_app):
        self.config = config
        self.simulation_app = simulation_app
        scenario_path = config['scenario_path']
        anim_people_command_dir = config['anim_people_waypoint_dir']
        environment_prim_path = config['environment_prim_path']
        random_command_generation = config['random_command_generation']
        headless = config['headless']
        with_people = config['with_people']
        use_generated_command_file = config['use_generated_command_file']
        tick_rate_hz = config['tick_rate_hz']
        enable_odometry = config['enable_odometry']

        import omni.kit.commands
        from omni.isaac.core import SimulationContext
        from omni.isaac.core.utils.nucleus import get_assets_root_path

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print(
                'Could not find Isaac Sim assets folder. Make sure you have an up to date local \
                Nucleus server or that you have a proper connection to the internet.'
            )
            self.simulation_app.close()
            exit()

        # usd_path = assets_root_path + scenario_path
        usd_path = scenario_path
        print(f'Opening stage {usd_path}...')

        # Load the stage
        omni.usd.get_context().open_stage(usd_path, None)

        # Wait two frames so that stage starts loading
        self.simulation_app.update()
        self.simulation_app.update()

        print('Loading stage...')
        from omni.isaac.core.utils.stage import is_stage_loading

        while is_stage_loading():
            self.simulation_app.update()
        print('Loading Complete')

        # If the scene with people is requested we build/rebuild the navmesh
        # We also point to the generated/custom command file for people animation if set by user
        if with_people:
            rebuild_nav_mesh()
            if not random_command_generation and use_generated_command_file:
                update_people_command_file_path(anim_people_command_dir)

        # Modify the omnigraph to get lidar point cloud published

        import omni.graph.core as og
        keys = og.Controller.Keys
        controller = og.Controller()

        carter_prim_path = '/World/Carter_ROS'

        if not enable_odometry:
            # Remove odometry from Carter
            og.Controller.edit(f'{carter_prim_path}/ActionGraph',
                            {keys.DELETE_NODES: ['isaac_compute_odometry_node',
                                                    'ros2_publish_odometry',
                                                    'ros2_publish_raw_transform_tree',
                                                    # 'ros2_publish_transform_tree_01',
                                                    ]})

        config_camera = config.get('camera', {})
        # new_camera()
        # Configure left camera
        configure_camera(carter_prim_path, controller, 'left', config_camera)
        # Configure right camera
        right_info = configure_camera(
            carter_prim_path, controller, 'right', config_camera)
        # Configure extra camera
        configure_camera(carter_prim_path, controller, 'extra', config_camera)
        configure_camera(carter_prim_path, controller, 'extra2', config_camera)

        stereo_offset = [-175.92, 0]
        # Set attribute
        right_info.get_attribute('inputs:stereoOffset').set(stereo_offset)

        time_dt = 1.0 / tick_rate_hz
        print(f'Running sim at {tick_rate_hz} Hz, with dt of {time_dt}')
        # Run physics at 60 Hz and render time at the set frequency to see the sim as real time
        self.simulation_context = SimulationContext(stage_units_in_meters=1.0,
                                            physics_dt=1.0 / 60,
                                            rendering_dt=time_dt)
        
    def start_simulation(self):
        with_people = self.config['with_people']
        random_command_generation = self.config['random_command_generation']
        environment_prim_path = self.config['environment_prim_path']
        anim_people_command_dir = self.config['anim_people_waypoint_dir']
        num_waypoints = self.config['num_waypoints']
        self.simulation_context.play()
        self.simulation_context.step()

        # Physics can only be seen when the scene is played
        # If we want to generate the command file for the people simulation we call the
        # create_people_commands method, which stores the command file and then we close the
        # simulation and point the user on how to use the generated file
        if with_people and random_command_generation:
            print('Creating human animation file...')
            create_people_commands(environment_prim_path,
                                anim_people_command_dir, 10, num_waypoints)
            print(
                'Human command file has been created at {}/human_human_cmd_file.txt'
                .format(anim_people_command_dir))
            print(
                'Please restart the simulation with --with_people and \n \
                --use_generated_command_file to use the generated command file in human simulation'
            )
            self.simulation_context.stop()
            self.simulation_app.close()
            
    def run_simulation(self, node_metrics):
        tick_rate_hz = self.config['tick_rate_hz']
        time_dt = 1.0 / tick_rate_hz
        # Indented format with 2 spaces for indentation
        pprint.pprint(self.config, indent=2)

        import omni.kit.viewport.utility as viewport_utils
        import rclpy

        # Run the sim
        last_frame_time = time.monotonic()
        while self.simulation_app.is_running():
            self.simulation_context.step()
            current_frame_time = time.monotonic()
            if current_frame_time - last_frame_time < time_dt:
                time.sleep(time_dt - (current_frame_time - last_frame_time))
            last_frame_time = time.monotonic()

            viewport_api = viewport_utils.get_active_viewport()
            # print(f"FPS {viewport_api.fps:.2f}", end="\r")
            # export_cycle_value(viewport_api.fps)
            node_metrics.viewpoint(viewport_api.fps)
            rclpy.spin_once(node_metrics, timeout_sec=0.0)

        self.simulation_context.stop()
        self.simulation_app.close()
# EOF
