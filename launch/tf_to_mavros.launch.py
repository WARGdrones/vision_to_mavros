import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# Your camera namespace
camera_name = 'rgb_camera'

# Location of configuration directory
config_dir = os.path.join(
    get_package_share_directory('vision_to_mavros'), 'cfg')
# print(config_dir)

# # Parameters file
# params_file = os.path.join(config_dir, 'default.yaml')
# print(params_file)

# # Camera calibration file
# config = 'file://' + os.path.join(config_dir, 'calibration.yaml')
# print(config)

# detect all 36h11 tags
# cfg_36h11 = {
#     'image_transport': 'raw',
#     'family': '36h11',
#     'size': 0.162
# }

'''
 Two steps alignment:
    1) r,p,y: align current camera frame with default camera frame (x forward, y left, z up)
    2) gamma: align default camera frame's x axis with world y axis
    Frontfacing:
        Forward, USB port to the right (default): r = 0,          p = 0,          y = 0,  gamma = -1.5707963
        Forward, USB port to the left           : r = 3.1415926,  p = 0,          y = 0,  gamma = -1.5707963
    Downfacing: you need to tilt the vehicle's nose up a little (not flat) when launch the T265 realsense-ros node, otherwise the initial yaw will be randomized, read here: https://github.com/IntelRealSense/librealsense/issues/4080
    Tilt the vehicle to any other sides and the yaw might not be as stable.
        Downfacing, USB port to the right :       r = 0,          p = -1.5707963, y = 0,  gamma = -1.5707963
        Downfacing, USB port to the left  :       r = 3.1415926,  p = -1.5707963, y = 0,  gamma = -1.5707963
        Downfacing, USB port to the back  :       r = -1.5707963, p = -1.5707963, y = 0,  gamma = -1.5707963
        Downfacing, USB port to the front :       r = 1.5707963,  p = -1.5707963, y = 0,  gamma = -1.5707963
    Default for this launch file: downfacing, USB port to the right
'''


def generate_launch_description():
    tf_to_mavros_node = Node(
        package='vision_to_mavros',
        executable='vision_to_mavros',
        output='screen',
        emulate_tty=True,
        name='vision_to_mavros',
        # namespace=camera_name,
        parameters=[
            {
                # target_frame_id must matches the tag’s name
                "target_frame_id": "36h11:1",
                # source_frame_id must matches the camera_frame used in apriltag_ros
                "source_frame_id": "rgb_camera_frame",
                # enable publishing precision landing messages
                "enable_precland": True,
                # precland_target_frame_id must matches the tag’s name
                "precland_target_frame_id": "36h11:1",
                # precland_camera_frame_id in  must matches camera_frame used in apriltag_ros
                "precland_camera_frame_id": "rgb_camera_frame",
                "roll_cam": 0.0,
                "pitch_cam": -1.5707963,
                "yaw_cam": 0.0,
                "gamma_world": -1.5707963,
                "output_rate": 10.0
                # 'vision_pose': '/mavros/vision_pose/pose',
            }
        ],
        # Remap outputs to the correct namespace
        remappings=[
            ('vision_pose', '/mavros/vision_pose/pose'),
            ('landing_raw', '/mavros/landing_target/raw')
        ]
    )

    return LaunchDescription([
        tf_to_mavros_node,
    ])
