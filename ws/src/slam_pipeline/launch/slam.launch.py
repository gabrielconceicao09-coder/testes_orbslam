from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # caminho do rviz config dentro do pacote
    rviz_config = os.path.join(
        get_package_share_directory('slam_pipeline'),
        'config',
        'slam.rviz'
    )

    # ================= CAMERA =================
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'pixel_format': 'RGB24',
            'image_size': [640, 480],
            'camera_frame_id': 'camera'
        }],
        remappings=[
            ('image_raw', '/image_raw')
        ]
    )

    # ================= ORB-SLAM3 =================
    orbslam_node = Node(
        package='slam_pipeline',
        executable='orbslam_node',
        name='orbslam_node',
        output='screen'
    )

    # ================= RVIZ =================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        camera_node,
        orbslam_node,
        rviz_node
    ])

