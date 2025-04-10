import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

from cybership_utilities.launch import anon


def generate_launch_description():

    vessel_name = "enterprise"
    vessel_model = "enterprise"

    node_utilities = launch_ros.actions.Node(
        package='tmr4243_utilities',
        executable='utility_node.py',
        name=f"utilities_{anon()}",
        namespace=vessel_name,
        output='screen'
    )

    description_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        "cybership_simulator"),
                    "launch",
                    "voyager.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("vessel_model", vessel_model),
            ("vessel_name", vessel_name),
            ("use_gui", "true"),
        ],
    )

    ld = launch.LaunchDescription()
    ld.add_action(node_utilities)
    ld.add_action(description_launch)

    return ld

