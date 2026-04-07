from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Motor Controller Node
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen'
        ),

        # 2. Ultrasonic Sensor 1 (Left)
        Node(
            package='rovultrasonic',
            executable='publishd1',
            name='ultrasonic_left',
            output='screen'
        ),

        # 3. Ultrasonic Sensor 2 (Right)
        # Note: Ensure your second sensor script is named 'publishd2' 
        # or matches your executable name in setup.py
        Node(
            package='rovultrasonic',
            executable='publishd2',
            name='ultrasonic_right',
            output='screen'
        ),

        # 4. The Brain (UONLY Node)
        Node(
            package='RovControl', # Change to your actual package name
            executable='uonly',
            name='uonly_logic',
            output='screen'
        )
    ])