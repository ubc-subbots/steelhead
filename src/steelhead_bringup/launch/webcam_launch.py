# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     camera = Node(
#         package='usb_cam',
#         executable='usb_cam_node_exe',
#         name='webcam',
#         parameters=[{
#             'video_device': '/dev/video0',   # Change if your webcam uses another device
#             'image_width': 640,
#             'image_height': 480,
#             'framerate': 30.0,
#             'pixel_format': 'yuyv'
#         }],
#         remappings=[
#             ('/image_raw', '/steelhead/drivers/webcam/image_raw')
#         ]
#     )

#     return LaunchDescription([camera])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'frame_rate': 30.0,
            'pixel_format_fourcc': 'MJPG',
            'image_transport': 'compressed'  # <--- publish compressed directly
        }],
        remappings=[
            ('/image_raw', '/steelhead/drivers/webcam/image_raw')
        ]
    )

    return LaunchDescription([camera_node])
