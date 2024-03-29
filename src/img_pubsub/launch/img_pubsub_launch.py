from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_path = '/home/atr/mech_devel/mechatroncis_ws/src/img_pubsub/config/img_pubsub_config.yaml'
    # config_path = 'package://img_pubsub/config/img_pubsub_config.yaml'
    return LaunchDescription([
        Node(
            package="img_pubsub",
            executable="img_pubsub_node",
            name="img_pubsub_node",
            output="screen",
            parameters=[config_path]
        ),
    ])