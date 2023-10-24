from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='sub_teleop',
        #    namespace='teleop_cmd',
        #    executable='teleop_twist_keyboard_vf',
        #    name='sim'
        #),
        Node(
            package='teleop',
            namespace='teleop_cmd',
            executable='teleop_twist_keyboard_vf_gazebo',
            name='sim2',
            remappings=[
                ('/teleop_cmd/rover_base2/command/mode', '/rover_base2/command/mode'),
                ('/teleop_cmd/rover_base2/cmd_vel', '/rover_base2/cmd_vel'),
                ('/teleop_cmd/rover_base2/command/pos_vel', '/rover_base2/command/pos_vel'),
                ('/teleop_cmd/keyboard/keypress', '/keyboard/keypress')
            ]
        )
    ])