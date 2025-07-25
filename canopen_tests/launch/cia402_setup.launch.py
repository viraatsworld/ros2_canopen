#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
import launch_ros
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():



    master_bin_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "config",
        "cia402_lifecycle",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""


    # Function Begins :-
    launch_mode = LaunchConfiguration("launch_mode")
    launch_mode_args=DeclareLaunchArgument(
            name="launch_mode",
            default_value=TextSubstitution(text='normal'),
            choices= ['normal','diagnostics','lifecycle'],
            description="select whether simulation or joint state publisher",
        )


    #Normal Function
    slave_eds_path = os.path.join(
        get_package_share_directory("canopen_tests"), "config", "cia402", "cia402_slave.eds"
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2",
            "node_name": "cia402_node_1",
            "slave_config": slave_eds_path,
        }.items(),

       condition= IfCondition(PythonExpression([
    "'", launch_mode, "' == 'diagnostics' or '",
    launch_mode, "' == 'lifecycle' or '",
    launch_mode, "' == 'normal'"
])),

    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "3",
            "node_name": "cia402_node_2",
            "slave_config": slave_eds_path,
        }.items(),

               condition=IfCondition(PythonExpression([
            "'", launch_mode, "' == 'diagnostics'"
        ])),

    )

    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "4",
            "node_name": "cia402_node_4",
            "slave_config": slave_eds_path,
        }.items(),

       condition=IfCondition(PythonExpression([
            "'", launch_mode, "' == 'diagnostics'"
        ])),
    )


    master_bin_path=""

    normal_device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),

        condition=IfCondition(PythonExpression([
            "'", launch_mode, "' == 'normal'"
        ])),

    )



    lifecycle_device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402_lifecycle",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402_lifecycle",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),

        condition=IfCondition(PythonExpression([
            "'", launch_mode, "' == 'lifecycle'"
        ])),
    )




    #Diagnostics

    diagnostics_device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402_diagnostics",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402_diagnostics",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),


condition=IfCondition(PythonExpression([
            "'", launch_mode, "' == 'diagnostics'"
        ])),
    )


    diagnostics_analyzer_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "launch",
        "analyzers",
        "cia402_diagnostic_analyzer.yaml",
    )

    diagnostics_aggregator_node = launch_ros.actions.Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        output="screen",
        parameters=[diagnostics_analyzer_path],

condition=IfCondition(PythonExpression([
            "'", launch_mode, "' == 'diagnostics'"
        ])),    )


    return LaunchDescription([launch_mode_args, slave_node_1, slave_node_2,slave_node_3, normal_device_container, lifecycle_device_container,diagnostics_device_container, diagnostics_aggregator_node ])
