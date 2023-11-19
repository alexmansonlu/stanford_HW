from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),

            # relay RVIZ goal pose to some other channel
            Node(
                executable="rviz_goal_relay.py",
                package="asl_tb3_lib",
                parameters=[
                    {"output_channel": "/cmd_nav"},
                ],
            ),
            # state publisher for turtlebot
            Node(
                executable="state_publisher.py",
                package="asl_tb3_lib",
            ),
            # student's navigator node
            Node(
                executable="navigator.py",
                package="autonomy_repo",
                parameters=[{"use_sim_time": use_sim_time}]
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]
                ),
                launch_arguments={
                    "config": PathJoinSubstitution(
                        [
                            FindPackageShare("autonomy_repo"),
                            "rviz",
                            "default.rviz",
                        ]
                    ),
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )


