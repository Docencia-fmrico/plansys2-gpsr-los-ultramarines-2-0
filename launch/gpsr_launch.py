# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_gpsr')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': example_dir + '/pddl/planning_domain_durative.pddl',
            'namespace':namespace
            }.items()
        )

    '''
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('br2_tiago'),
            'launch',
            'sim.launch.py'))
        )
    '''
    ###########
    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('br2_tiago'),
            'launch',
            'sim.launch.py'))
        )
    ###########

    # Specify the actions
    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/bt_xml/move.xml'
          }
        ])
    
    take_stuff_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='PickItem',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pickitem',
            'publisher_port': 1670,
            'server_port': 1671,
            'bt_xml_file': example_dir + '/bt_xml/PickItem.xml'
          }
        ])

    release_stuff_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='DropItem',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'dropitem',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/bt_xml/DropItem.xml'
          }
        ])

    close_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='CloseDoor',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'closedoor',
            'publisher_port': 1674,
            'server_port': 1675,
            'bt_xml_file': example_dir + '/bt_xml/CloseDoor.xml'
          }
        ])
    
    open_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='OpenDoor',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'opendoor',
            'publisher_port': 1676,
            'server_port': 1677,
            'bt_xml_file': example_dir + '/bt_xml/OpenDoor.xml'
          }
        ])
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    # Declare the launch options

    ###########
    ld.add_action(simulation_cmd)
    ld.add_action(navigation_cmd)
    ##########
    
    ld.add_action(plansys2_cmd)
    '''
    ld.add_action(gazebo_cmd)
    '''
    ld.add_action(move_cmd)
    ld.add_action(take_stuff_cmd)
    ld.add_action(release_stuff_cmd)
    ld.add_action(open_door_cmd)
    ld.add_action(close_door_cmd)
    #####

    return ld