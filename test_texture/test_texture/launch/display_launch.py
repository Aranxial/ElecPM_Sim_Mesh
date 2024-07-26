from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Define the path to your xacro file
    xacro_path = '/home/siva/Elec PM/ros2_ws_text/src/test_texture/urdf/texture.xacro'
    
    # Process xacro file to get URDF content
    urdf_content = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
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
        #     arguments=['-d', '/home/siva/Elec PM/ros2_ws_text/src/test_texture/rviz/your_config.rviz']
        # )
    ])

