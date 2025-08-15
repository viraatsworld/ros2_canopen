Running Examples
================

In order to tryout the library a few examples are provided in the ``canopen_tests`` directory.
You can run them if you have :ref:`started the vcan0 interface <quick-start-setup-can-controller>`.

Service Interface
---------------------


The launch file supports three modes of operation, controlled via the ``launch_mode`` argument:

1. **Normal** (default)
2. **Lifecycle**
3. **Diagnostics**

Normal Mode (default)
~~~~~~~~~~~~~~~~~~~~~

This is the default mode used when no ``mode`` is specified.

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py mode:=normal

Lifecycle Mode
~~~~~~~~~~~~~~~~~~~

Enables lifecycle management of nodes. Use this mode if you want nodes to follow a managed lifecycle state machine.

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py mode:=lifecycle

Diagnostics Mode
~~~~~~~~~~~~~~~~~~~

Launches the nodes with diagnostics features enabled for monitoring and debugging.

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py mode:=diagnostics

Proxy Setup
~~~~~~~~~~~~~~~~~~~
Just like the cia402 drivers, we can test proxy based communication with CANopen nodes

.. code-block:: bash

    #normal
    ros2 launch canopen_tests proxy_setup.launch.py
    #lifecycle
    ros2 launch canopen_tests proxy_setup.launch.py mode:=lifecycle
    #diagnostics
    ros2 launch canopen_tests proxy_setup.launch.py mode:=diagnostics


ROS2 Control
-----------------
These launch files uses ROS 2 Control with a simulated robot defined in a XACRO-based URDF. They starts the controller manager, publishes the robot state, spawns controllers, and launches fake CANopen slave nodes to simulate actuator behavior.


CANopen System Setup
~~~~~~~~~~~~~~~~~~~~

You can launch either the proxy CANopen system or the CiA402 drivers control system with a ``mode`` argument.

Proxy CANopen System (default)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    ros2 launch canopen_tests canopen_system.launch.py mode:=canopen_system
    #or
    ros2 launch canopen_tests canopen_system.launch.py


CiA402 Motion Control System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    ros2 launch canopen_tests canopen_system.launch.py mode:=cia402_system

Robot Setup
~~~~~~~~~~~~~~~~~~~

Basic implementation with slave nodes

.. code-block:: bash

    ros2 launch canopen_tests robot_control_setup.launch.py
