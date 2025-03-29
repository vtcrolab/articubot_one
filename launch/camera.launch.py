# import os

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():



#     return LaunchDescription([

#         Node(
#             package='v4l2_camera',
#             executable='v4l2_camera_node',
#             output='screen',
#             namespace='camera',
#             parameters=[{
#                 'image_size': [640,480],
#                 'time_per_frame': [1, 6],
#                 'camera_frame_id': 'camera_link_optical'
#                 }]
#     )
#     ])


import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RealSense Camera Node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'enable_depth': True,
                'enable_color': True,
                'enable_pointcloud': True,
                'depth_width': 640,
                'depth_height': 480,
                'color_width': 640,
                'color_height': 480,
                'camera_frame_id': 'camera_link',
                'pointcloud.enable': True,
            }],
            remappings=[
                ('/camera/camera/color/image_raw', '/camera/color/image_raw')
            ]
        ),
        Node(
            package='articubot_one',  # Replace with your actual package name
            executable='color_detection_node',  # Name of the executable
            output='screen',
            namespace='color_detection',
        )
    ])
