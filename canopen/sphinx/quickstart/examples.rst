Running Examples
================

In order to tryout the library a few examples are provided in the ``canopen_tests`` directory.
You can run them if you have :ref:`started the vcan0 interface <quick-start-setup-can-controller>`.

Service Interface
---------------------
------------

The launch file supports three modes of operation, controlled via the `launch_mode` argument:

1. **Normal** (default)
2. **Lifecycle**
3. **Diagnostics**

Normal Mode
------------

This is the default mode used when no `launch_mode` is specified.

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py launch_mode:=normal

Lifecycle Mode
------------

Enables lifecycle management of nodes. Use this mode if you want nodes to follow a managed lifecycle state machine.

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py launch_mode:=lifecycle

Diagnostics Mode
------------

Launches the nodes with diagnostics features enabled for monitoring and debugging.

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py launch_mode:=diagnostics


ROS2 Control
------------
------------

Proxy Setup
,,,,,,,,,,,

.. code-block:: bash

   ros2 launch canopen_tests canopen_system.launch.py


CiA402 Setup
,,,,,,,,,,,,

.. code-block:: bash

   ros2 launch canopen_tests cia402_system.launch.py


Robot Setup
,,,,,,,,,,,,

.. code-block:: bash

    ros2 launch canopen_tests robot_control_setup.launch.py
