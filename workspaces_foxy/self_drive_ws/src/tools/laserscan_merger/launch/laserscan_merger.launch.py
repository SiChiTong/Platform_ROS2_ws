from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laserscan_merger',            
            executable='laserscan_merger',
            #namespace='laserscan_merger_1',
            name='laserscan_merger',
            parameters=[{
                "destination_frame" : "robot_base",
                "cloud_destination_topic" : "/cloud",
                "scan_destination_topic" : "/scan",
                "laserscan_topics" : "/scan_1 /scan_2",
                "angle_min" : -3.14,
                "angle_max" : 3.14,
                "angle_increment" : 0.0058,                
                "scan_time" : 0.033,
                "range_min" : 0.2,
                "range_max" : 25.0,
                "verbose" : False,
            }],
        )        
    ])
    
