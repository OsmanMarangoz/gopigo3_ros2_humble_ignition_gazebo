# In gopigo3_simulation/launch/gopigo3_spawn.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # force cpu for Gpu_lidar
    set_mesa_override = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.3'
    )
    set_libgl_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='true'
    )
    # Get paths to the necessary packages and files
    package_name = 'gopigo3_simulation'
    pkg_share = get_package_share_directory(package_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # --- Robot Description ---
    # Process the URDF file from xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'gopigo3.urdf.xacro') # Renaming to .xacro is good practice
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Gazebo Simulation ---
    # Set the path to the world file
    default_world = os.path.join(pkg_share, 'worlds', 'empty.world')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to the world file to load'
    )

    # Add the gopigo3_simulation/models path to the Gazebo resource path
    # This is crucial for Gazebo to find your robot's meshes and other assets
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    # Launch Gazebo Sim - server and client together
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        # Pass gz_args to the gz_sim.launch.py script
        launch_arguments={'gz_args': ['-r -v4 ', world]}.items()
    )

    # --- Nodes to Launch ---
    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Spawner
    # The 'create' executable spawns a model in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Spawn from this topic
            '-name', 'gopigo3',          # Name of the robot in simulation
            '-allow_renaming', 'true'    # Avoids errors if a robot with the same name exists
        ],
        output='screen'
    )
    # 3. ROS GZ Bridge node
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Create and return the LaunchDescription object
    return LaunchDescription([
        world_arg,
        set_model_path,

        set_mesa_override,
        set_libgl_software,

        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        ros_gz_bridge
    ])