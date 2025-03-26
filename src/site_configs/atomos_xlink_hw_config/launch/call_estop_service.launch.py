import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=["sudo /bin/sh -c 'sleep 5; ros2 service call /estop_reset std_srvs/src/Empty {}'"],
            output="screen"
        )
    ])
