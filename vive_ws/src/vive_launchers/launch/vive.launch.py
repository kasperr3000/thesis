import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import math


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vive_tracker',
            executable='vive_node',
            name='steamlaunch_and_map',
            prefix="/home/vintecc/vive_ws/src/vive_launchers/scripts/find_steam_runtime.sh",
            output='screen'
        ),
        launch_ros.actions.Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["-0.035","-0.030","1.09", str(-math.pi/2),"0",str(math.pi/2), "ROOT", "vr_base"]
        )
        
    ])



'''
launch_ros.actions.Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0","0","0.1", "0","0","1.56", "test", "ROOT"]
        )
        #str(math.pi/2)
        
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vive_launchers',
            executable='vive_node',
            name='steamlaunch',
            prefix=['xterm -e $(find vive_ros)/scripts/find_steam_runtime.sh'],
            output='screen'
        )
    ])
'''