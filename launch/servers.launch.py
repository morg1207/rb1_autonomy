from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #~~~~~~~~~~~~~~~~~ PACKAGE ~~~~~~~~~~~~~~~~~~~~

    package_name = 'rb1_autonomy'

    #~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~

    arg_robot_mode = DeclareLaunchArgument(
        'robot_mode',
        default_value='sim_robot',
        choices=['sim_robot', 'real_robot'],
        description='Use for simlation robot or real robot'
    )

    config_robot_mode = LaunchConfiguration('robot_mode')

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~

    package_share = get_package_share_directory(package_name)
    server_find_object_yaml = PathJoinSubstitution([package_share, 'config',config_robot_mode,'server_find_object.yaml'])
    server_approach_shelf_yaml = PathJoinSubstitution([package_share, 'config',config_robot_mode,'server_approach_shelf.yaml'])
    server_init_localization_yaml = PathJoinSubstitution([package_share, 'config',config_robot_mode,'server_init_localization.yaml'])

    #~~~~~~~~~~~~~~~~~~~~~~~~ NODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~

    server_find_object = Node(
        name='server_find_object',
        executable='main_server_find_object',
        package='rb1_autonomy',
        parameters=[server_find_object_yaml],
        emulate_tty=True,
        arguments=['--ros-args', 
                   #'--log-level', 'Laser:=DEBUG',
                   '--log-level','server_find_object:=DEBUG'],
    )

    server_approach_shelf = Node(
        name='server_approach_shelf',
        executable='main_server_approach_shelf',
        package='rb1_autonomy',
        parameters=[server_approach_shelf_yaml],
        emulate_tty=True,
        arguments=['--ros-args', 
                   '--log-level','server_approach_shelf:=DEBUG'],
    )

    server_init_localization = Node(
        name='server_init_localization',
        executable='main_server_init_localization',
        package='rb1_autonomy',
        parameters=[server_init_localization_yaml],
        emulate_tty=True,
        arguments=['--ros-args', 
                   '--log-level','server_init_localization:=DEBUG'],
    )

    return LaunchDescription([
        arg_robot_mode,

        server_find_object,
        server_approach_shelf,
        server_init_localization
    ])