# #!/usr/bin/env python3

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():


#     robot_description_content = C
#     #     [
#     #         PathJoinSubstitution([FindExecutable(name="xacro")]),
#     #         " ",
#     #         PathJoinSubstitution(
#     #             [
#     #                 FindPackageShare("arena_description"),
#     #                 "urdf",
#     #                 "arena.xacro",
#     #             ]
#     #         ),

#     #     ]
#     # )
#     robot_description = {"robot_description": robot_description_content}

#     return LaunchDescription([
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[robot_description]),
#     ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'urdf/arena.xacro'
  urdf = '/home/nikhil/Documents/simulation_ws/src/my_package/description/example_robot.urdf.xacro'

  print("urdf_file_name : {}".format(urdf_file_name))

#   urdf = os.path.join(
#       get_package_share_directory('ros2_sim_pkg'),
#       urdf_file_name)

  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf])
  ])