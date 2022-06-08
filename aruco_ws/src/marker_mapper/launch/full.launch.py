import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
import sys
import math
from ament_index_python.packages import get_package_prefix
from pathlib import Path
import pathlib

def generate_launch_description():
    #go to colcon_ws/install/
    path_to_params = Path(__file__).parent.parent.parent.parent.parent
    #go to colcon_ws/install/camera_info/share/camera_info
    path_to_params = str(path_to_params / 'camera_info' / 'share' / 'camera_info' )
    

    return launch.LaunchDescription([
        launch_ros.actions.Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name= 'aruco_camera',
        # namespace=ns,
        parameters=[path_to_params + '/params.yaml']
        ),
        launch_ros.actions.Node(
            package='bridger',
            executable='bridge',
            name='ArUcoVisionPose'
        ),
        launch_ros.actions.Node(
            package='marker_mapper',
            executable='mapper',
            name='ArUcoMapper'
        )#,
        # launch_ros.actions.Node(
        #     package = "tf2_ros", 
        #     executable = "static_transform_publisher",
        #     arguments = ["0.15","0","0.15", "0",str(-math.pi),str(-math.pi/2), "ROOT", "world"]
        # )
        
    ])
