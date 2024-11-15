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
    autonomy_yaml = PathJoinSubstitution([package_share, 'config',config_robot_mode,'autonomy.yaml'])

    #~~~~~~~~~~~~~~~~~~~~~~~~ NODE ~~~~~~~~~~~~~~~~~~~~~~~~~~~

    autonomy = Node(
        executable='main_autonomy',
        package='rb1_autonomy',
        parameters=[autonomy_yaml],
        emulate_tty=True,
        output='screen',
        arguments=['--ros-args', 
                   '--log-level','bt_find_object_client:=DEBUG',
                   '--log-level','bt_publish_transform:=DEBUG',
                   '--log-level','node_bt:=DEBUG',
                   '--log-level','bt:=DEBUG',
                   '--log-level','bt_turn_robot:=DEBUG'],
    )

    return LaunchDescription([

        arg_robot_mode,

        autonomy
    ])