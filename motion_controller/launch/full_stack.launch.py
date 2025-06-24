from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    """
    Generate the launch description for the system.

    Returns:
        LaunchDescription: Configures the full stack with optional GUI and custom controller.
    """
    headless = LaunchConfiguration("headless")
    sim = LaunchConfiguration(
        "sim"
    )  # used with the simulated robot mode (not implemented)
    controller_exec = LaunchConfiguration("controller_exec")

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="If true, do not launch the GUI interface",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="If true, launch in simulated robot mode (not implemented)",
            ),
            DeclareLaunchArgument(
                "controller_exec",
                default_value="controller",
                description="If specified then use that else use the default one",
            ),
            # Clock pose publisher (always runs)
            Node(
                package="clock_pose_issuer",
                executable="clock_publisher",
                name="clock_pose_publisher",
                output="screen",
            ),
            # GUI pose publisher (only if headless is false)
            Node(
                package="gui_pose_issuer",
                executable="gui_publisher",
                name="gui_pose_publisher",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", headless, "' == 'false'"])
                ),
            ),
            # Motion controller
            Node(
                package="motion_controller",
                executable=controller_exec,
                name="motion_controller",
                output="screen",
            ),
            # Placeholder for simulator launch
            # Node(
            #     package='sim_package',
            #     executable='sim_node',
            #     name='robot_simulator',
            #     output='screen',
            #     condition=IfCondition(PythonExpression(["'", sim, "' == 'true'"]))
            # )
        ]
    )
