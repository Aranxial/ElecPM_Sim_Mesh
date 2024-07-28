# Adding Aruco-tags on the Cube
The package ``test_texture`` was made initially to check whether importing meshes to ``gazebo`` via ``.dae`` files was possible or not.

The final package is ``aruco_cube``. Download and place it in the ``src`` directory of your ROS workspace and build it using ``colcon build --symlink-install`` and source the setup for the workspace ``source install/setup.bash``.

After that run ``ros2 launch aruco_cube display_launch.py`` **(default)** which creates the ``robot_state_publisher``. In another terminal, open Ignition: ``ign gazebo``. Now, open one more terminal and type: ``ros2 run ros_gz_sim create -topic robot_description -n robo_name`` which spawns the Cube with Aruco tags in its faces in ignition.

The tags you can see would be 011.png to 016.png **(default)**. If you need another set of aruco tags, use the command: ``ros2 launch aruco_cube display_launch.py number:=$(number)``. $(number) denotes the cube number (eg: 2 for 021...026.png, 15 for 151...156.png as aruco tags).
