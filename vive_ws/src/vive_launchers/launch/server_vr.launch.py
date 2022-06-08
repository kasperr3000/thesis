import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return launch.LaunchDescription([
        SetEnvironmentVariable(name="OPENVR", value=[EnvironmentVariable('HOME'), '/libraries/openvr']),
        SetEnvironmentVariable(name="STEAM", value=[EnvironmentVariable('HOME'), '/.local/share/Steam']),
        SetEnvironmentVariable(name="STEAMVR", value=[EnvironmentVariable('HOME'), '/.steam/steam/steamapps/common/SteamVR']),

        SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=[  EnvironmentVariable('LD_LIBRARY_PATH')
                                                                , ':', EnvironmentVariable('HOME'), '/.local/share/Steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu'
                                                                , ':', EnvironmentVariable('HOME'), '/.steam/steam/steamapps/common/SteamVR/bin/linux64'
                                                                , ':', EnvironmentVariable('HOME'), '/.steam/steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux64']),
        launch_ros.actions.Node(
            package='vive_launchers',
            executable='launch_servervr.sh',
            name='server_vr',
            output='screen'
        )
    ])