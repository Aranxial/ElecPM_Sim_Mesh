from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the launch argument for the number
    number_arg = DeclareLaunchArgument(
        'number',
        default_value='1',
        description='Number to use for updating DAE textures'
    )

    # Get the value of the 'number' argument
    number = LaunchConfiguration('number')

    # Find the package share directory
    package_name = 'aruco_cube'
    package_share_directory = get_package_share_directory(package_name)

    # Define the path to your xacro file using the package directory
    xacro_path = os.path.join(package_share_directory, 'urdf', 'robot.xacro')

    # Define the path to your Python script using the package directory
    script_path = os.path.join(package_share_directory, 'codes', 'update_dae.py')

    # Process xacro file to get URDF content
    urdf_content = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
        # Declare the launch argument
        number_arg,
        # Run the Python script to update the DAE textures
        ExecuteProcess(
            cmd=['python3', script_path, LaunchConfiguration('number')],
            output='screen'
        ),
        # Launch the robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content
            }]
        ),
        # Uncomment and update if you want to include RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', os.path.join(package_share_directory, 'rviz', 'your_config.rviz')]
        # )
    ])

