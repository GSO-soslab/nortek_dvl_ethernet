import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot = 'test'
    
    param = os.path.join(
        get_package_share_directory('nortek_dvl'),
        'config',
        'param.yaml'
        )
    
    return LaunchDescription([
        # Nortek DVL
        Node(
            package="nortek_dvl",
            executable="nortek_dvl_node",
            name="nortek_dvl_node",
            output="screen",
            namespace=robot,
            parameters=[param],
            emulate_tty=True        
        ),

        # ExecuteProcess(
        #     cmd=[
        #         "ros2",
        #         "bag",
        #         "play",
        #         "/home/soslab_lin/Develop/data/fls_reconstruction/rosbag2_2025_04_25-16_17_10_0.mcap",
        #     ],
        #     output="screen",
        # )        

    ])