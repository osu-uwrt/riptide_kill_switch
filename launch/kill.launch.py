import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    # Read in the vehicle's namespace through the command line or use the default value one is not provide
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "robot", 
            default_value="tempest",
            description="Name of the vehicle",
        ),

        # create the nodes    
        launch_ros.actions.Node(
            package='robostop',
            executable='physical_kill_switch',
            name='physical_kill_switch',
            respawn=True,
            output='screen',
            namespace=LaunchConfiguration("robot")
        )
    ])
