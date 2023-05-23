ROS2 Control
=============================


Hardware Interface
------------------
This package provides multiple hardware interfaces for testing. Mainly the following:

- canopen_ros2_control/CanopenSystem: A system interface for ProxyDrivers
- canopen_ros2_control/Cia402System: A system interface for Cia402Drivers
- canopen_ros2_control/RobotSystem: A system interface for Cia402Drivers in a robot configuration (under development)

Robot System Interface
''''''''''''''''''''''

The robot system interface takes a number of inputs from the robot description (urdf).
It will make the Cia402Drivers available via the ros2_control hardware interface.
The bus has to still be defined in the bus.yml file. In the urdf you can the choose the
CANopen nodes that have a Cia402Driver attached to them.

The ros2_control interface only works with non-lifecycle drivers right now.
For each joint in your urdf you can choose the attached CANopen device by using the
``device_name`` parameter. The ``device_name`` parameter is the CANopen device and it described in ``bus.yml`` file.

.. code-block:: xml

    <ros2_control name="${name}" type="system">
        <hardware>
            <plugin>canopen_ros2_control/RobotSystem</plugin>
            <param name="bus_config">[path to bus.yml]</param>
            <param name="master_config">[path to master.dcf]</param>
            <param name="can_interface_name">[can interface to be used]</param>
            <param name="master_bin">[master.bin if it exists]</param>
        </hardware>
        <joint name="joint1">
            <param name="device_name">joint_1</param>
            ...
        </joint>
        <joint name="joint2">
            <param name="device_name">joint_2</param>
            ...
        </joint>
    </ros2_control>

The robot system interface adds few more configuration parameters to the ``bus.yml`` file.

.. code-block:: yaml

    [...]

    defaults:
        [...]
        position_mode: [value]
        [or]
        velocity_mode: [value]
        [or]
        effort_mode: [value]
        [...]

    nodes:
        joint_1:
            node_id : [value]
        joint_2:
            node_id : [value]

For more information about value to be used for the different modes, please refer to the `API documentation <https://ros-industrial.github.io/ros2_canopen/api/classros2__canopen_1_1MotorBase.html>`_.

.. note::

    You can find an example for the configuration in the ``canopen_tests`` package under robot_control.


ROS2 Controllers
----------------
This package provides multiple controllers for testing. Mainly the following:

- canopen_ros2_controllers/Cia402RobotController: Works with Robot System Interface
- canopen_ros2_controllers/Cia402DeviceController: Works with Cia402System
- canopen_ros2_controllers/CanopenProxyController: Works with CanopenSystem and Cia402System

Robot Controller
''''''''''''''''

The robot controller enables bringing up the different joints of the robot automatically
by using the ros2_controller lifecycle. There is no need for further action, once the
controller is activated, the drives are ready to be used.

The robot controller can be configured in the ros2_controllers.yaml with the following
parameters:

.. code-block:: yaml

    robot_controller:
        ros__parameters:
            joints:  # joints that are controlled by the controller
            - joint1
            - joint2
            operation_mode: 1 # operation mode of the controller
            command_poll_freq: 5 # frequency with which the controller polls for command feedback
