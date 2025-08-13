from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='src/triton_gate/model_weights/arvp_front.pt',
        description='Path to the YOLO model file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug mode for visualization'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for YOLO detections'
    )

    # YOLO Gate Detector Node
    yolo_gate_detector = Node(
        name='yolo_gate_detector',
        namespace='/triton/gate',
        package='triton_gate',
        executable='yolo_gate_detector.py',
        parameters=[
            {'model_path': LaunchConfiguration('model_path')},
            {'debug': LaunchConfiguration('debug')},
            {'confidence_threshold': LaunchConfiguration('confidence_threshold')}
        ],
        output='screen'
    )

    # Add actions to launch description
    ld.add_action(model_path_arg)
    ld.add_action(debug_arg)
    ld.add_action(confidence_threshold_arg)
    ld.add_action(yolo_gate_detector)

    return ld