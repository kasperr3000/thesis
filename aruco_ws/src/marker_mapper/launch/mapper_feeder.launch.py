import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bridger',
            executable='bridge',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/vintecc/aruco_ws/bag_files/recht/rosbag2_2022_03_18-15_34_11', '-l'],
            output='screen'
        )
    ])