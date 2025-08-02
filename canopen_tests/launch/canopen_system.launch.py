# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Lovro Ivanov lovro.ivanov@gmail.com

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")

    #launch mode canopen_system or cia402_system
    mode = LaunchConfiguration("mode")


    # bus configuration
    bus_config_package = LaunchConfiguration("bus_config_package")
    #dynamically set based on mode
    config_directory =  PathJoinSubstitution(["config", mode])
    bus_config_file = LaunchConfiguration("bus_config_file")
    # bus configuration file full path
    bus_config = PathJoinSubstitution(
        [FindPackageShare(bus_config_package), config_directory, bus_config_file]
    )

    # master configuration
    master_config_package = LaunchConfiguration("master_config_package")
    master_config_file = LaunchConfiguration("master_config_file")
    # master configuration file full path
    master_config = PathJoinSubstitution(
        [FindPackageShare(master_config_package), config_directory, master_config_file]
    )

    # can interface name
    can_interface_name = LaunchConfiguration("can_interface_name")

    # robot description stuff
    description_package = LaunchConfiguration("description_package")

    # Setting urdf based on mode
    string_mode = LaunchConfiguration("mode").perform(context)
    description_file =  f"{string_mode}.urdf.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", mode, description_file]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "bus_config:=",
            bus_config,
            " ",
            "master_config:=",
            master_config,
            " ",
            "can_interface_name:=",
            can_interface_name,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ros2 control configuration
    ros2_control_config_package = LaunchConfiguration("ros2_control_config_package")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    # ros2 control configuration file full path
    ros2_control_config = PathJoinSubstitution(
        [
            FindPackageShare(ros2_control_config_package),
            config_directory,
            ros2_control_config_file,
        ]
    )

    # nodes to start are listed below
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_control_config],
        output="screen",
    )

    #add a delay to allow slaves to spawn first
    delayed_control_node = TimerAction(
    period=5.0,  # Delay in seconds (adjust as needed)
    actions=[control_node],
        )

    # load one controller just to make sure it can connect to controller_manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    canopen_proxy_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["node_1_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'canopen_system'"
        ])),
    )


    cia402_device_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cia402_device_1_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'cia402_system'"
        ])),
    )

    forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'cia402_system'"
        ])),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # hardcoded slave configuration form test package
    canopen_system_slave_config = PathJoinSubstitution(
        [FindPackageShare(master_config_package), "config/canopen_system", "simple.eds"]
    )

    cia402_system_slave_config = PathJoinSubstitution(
        [FindPackageShare("canopen_tests"), "config/cia402", "cia402_slave.eds"]
    )

    canopen_system_slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "basic_slave.launch.py"]
    )

    cia402_system_slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(canopen_system_slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_2",
            "slave_config": canopen_system_slave_config,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'canopen_system'"
        ])),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(canopen_system_slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_3",
            "slave_config": canopen_system_slave_config,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'canopen_system'"
        ])),
    )

    #cia402system
    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cia402_system_slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "cia402_node_1",
            "slave_config": cia402_system_slave_config,
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'cia402_system'"
        ])),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        slave_node_1,
        slave_node_2,
        slave_node_3,
        delayed_control_node,
        forward_position_controller,
        canopen_proxy_controller_spawner,
        cia402_device_controller_spawner,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "name", description="robot name", default_value="canopen_test_system"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("prefix", description="Prefix.", default_value="")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where urdf file is stored.",
            default_value="canopen_tests",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="mode",
            default_value=TextSubstitution(text="canopen_system"),
            choices=["canopen_system", "cia402_system"],
            description="Select system mode"
        ),

    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "canopen_system_description_file",
            description="Name of the urdf file.",
            default_value="canopen_system.urdf.xacro",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cia402_system_description_file",
            description="Name of the urdf file.",
            default_value="cia402_system.urdf.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_package",
            default_value="canopen_tests",
            description="Path to ros2_control configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value="ros2_controllers.yaml",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_package",
            default_value="canopen_tests",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_file",
            default_value="bus.yml",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_package",
            default_value="canopen_tests",
            description="Path to master configuration file (*.dcf)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_file",
            default_value="master.dcf",
            description="Path to master configuration file (*.dcf)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="Interface name for can",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
