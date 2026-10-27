from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_frame = "steelhead_auv/base_link/base_link/front_camera"
    camera_remappings = [
        ("rgb/image", "/steelhead/drivers/front_camera/image_raw"),
        ("rgb/camera_info", "/steelhead/drivers/front_camera/camera_info"),
        ("depth/image", "/steelhead/drivers/front_camera/depth/image_raw"),
    ]

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="front_camera_mount_tf",
        arguments=[
            "--x",
            "0.45",
            "--y",
            "-0.06",
            "--z",
            "0",
            "--roll",
            "-1.5708",
            "--pitch",
            "0",
            "--yaw",
            "-1.5708",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            camera_frame,
        ],
        output="screen",
    )

    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "subscribe_depth": True,
                "approx_sync": True,
                "use_sim_time": True,
            }
        ],
        remappings=camera_remappings,
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "subscribe_depth": True,
                "subscribe_odom_info": True,
                "approx_sync": True,
                "use_sim_time": True,
                "Mem/IncrementalMemory": "true",
            }
        ],
        remappings=camera_remappings,
        arguments=["--delete_db_on_start"],
    )

    return LaunchDescription([camera_mount_tf, rgbd_odometry, rtabmap])
