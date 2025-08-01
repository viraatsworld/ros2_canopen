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
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


    mode = LaunchConfiguration("mode")
    mode_args=DeclareLaunchArgument(
            name="mode",
            default_value=TextSubstitution(text='normal'),
            choices= ['normal','diagnostics','lifecycle'],
            description="select whether simulation or joint state publisher",
        )

    #normal
    normal_slave_eds_path = os.path.join(
        get_package_share_directory("canopen_tests"), "config", "simple", "simple.eds"
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_1",
            "slave_config": normal_slave_eds_path,
        }.items(),
        condition= IfCondition(PythonExpression([
    "'", mode, "' == 'diagnostics' or '",
    mode, "' == 'normal'"
        ])),

    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_2",
            "slave_config": normal_slave_eds_path,
        }.items(),
        condition= IfCondition(PythonExpression([
    "'", mode, "' == 'diagnostics' or '",
    mode, "' == 'normal'"
        ])),

    )


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
                "simple",
                "master.dcf",
            ),
            "master_bin": "",
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "simple",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),

        condition= IfCondition(PythonExpression([
    "'",mode, "' == 'normal'"
        ])),
    )

    # Delay master start by 3 seconds
    delayed_normal_device_container = TimerAction(
        period=3.0,
        actions=[normal_device_container],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'normal'"
        ])),
    )



    #lifecycle
    lifecycle_slave_eds_path = os.path.join(
        get_package_share_directory("canopen_tests"), "config", "simple_lifecycle", "simple.eds"
    )


    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2",
            "node_name": "slave_node_1",
            "slave_config": lifecycle_slave_eds_path,
        }.items(),
                condition= IfCondition(PythonExpression([
    "'", mode, "' == 'lifecycle'"
        ])),
    )

    slave_node_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "3",
            "node_name": "slave_node_2",
            "slave_config": lifecycle_slave_eds_path,
        }.items(),
                condition= IfCondition(PythonExpression([
    "'", mode, "' == 'lifecycle'"
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
                "simple_lifecycle",
                "master.dcf",
            ),
            "master_bin": "",
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "simple_lifecycle",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),

        condition= IfCondition(PythonExpression([
            "'", mode, "' == 'lifecycle'"
        ])),
    )

        # Delay master start by 3 seconds
    delayed_lifecycle_device_container = TimerAction(
        period=3.0,
        actions=[lifecycle_device_container],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'lifecycle'"
        ])),
    )

    #diagnostic
    #share same .eds file with simple
    diagnostic_device_container = IncludeLaunchDescription(
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
                "simple_diagnostics",
                "master.dcf",
            ),
            "master_bin": "",
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "simple_diagnostics",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),

        condition= IfCondition(PythonExpression([
    "'", mode, "' == 'diagnostics'"
        ])),
    )

    # Delay master start by 3 seconds -to bootup slave nodes
    delayed_diagnostic_device_container = TimerAction(
        period=3.0,
        actions=[diagnostic_device_container],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'diagnostics'"
        ])),
    )

    diagnostics_analyzer_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "launch",
        "analyzers",
        "proxy_diagnostic_analyzer.yaml",
    )

    diagnostics_aggregator_node = launch_ros.actions.Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        output="screen",
        parameters=[diagnostics_analyzer_path],
                condition= IfCondition(PythonExpression([
    "'",mode, "' == 'diagnostics'"
        ])),
    )


    print(
        os.path.join(
            get_package_share_directory("canopen_tests"),
            "config",
            "proxy_write_sdo",
            "master.dcf",
        )
    )

    return LaunchDescription([mode_args,slave_node_1, slave_node_2, delayed_normal_device_container,slave_node_3, slave_node_4, delayed_lifecycle_device_container, delayed_diagnostic_device_container, diagnostics_aggregator_node])
