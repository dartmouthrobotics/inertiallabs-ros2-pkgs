from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    container_name = LaunchConfiguration("container_name")
    ins_url = LaunchConfiguration("ins_url")
    ins_output_format = LaunchConfiguration("ins_output_format")
    frame_id = LaunchConfiguration("frame_id")
    enable_realtime_priority = LaunchConfiguration("enable_realtime_priority")
    use_device_time = LaunchConfiguration("use_device_time")
    publisher_queue_depth = LaunchConfiguration("publisher_queue_depth")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name",
                default_value="shared_container",
                description="Existing component container name",
            ),
            DeclareLaunchArgument(
                "ins_url",
                default_value="serial:/dev/ttyUSB0:921600",
                description="INS connection URL",
            ),
            DeclareLaunchArgument(
                "ins_output_format",
                default_value="149",
                description="INS output format",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Frame ID for INS message headers (if empty, serial number is used)",
            ),
            DeclareLaunchArgument(
                "enable_realtime_priority",
                default_value="false",
                description="Enable realtime scheduling for INS driver",
            ),
            DeclareLaunchArgument(
                "use_device_time",
                default_value="true",
                description="Use device time for INS timestamps",
            ),
            DeclareLaunchArgument(
                "publisher_queue_depth",
                default_value="200",
                description="Publisher QoS queue depth for INS topics",
            ),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package="inertiallabs_ins",
                        plugin="IL_INS",
                        name="ins",
                        namespace="miniAHRS",
                        parameters=[
                            {
                                "ins_url": ParameterValue(ins_url, value_type=str),
                                "ins_output_format": ParameterValue(
                                    ins_output_format, value_type=int
                                ),
                                "frame_id": ParameterValue(frame_id, value_type=str),
                                "enable_realtime_priority": ParameterValue(
                                    enable_realtime_priority, value_type=bool
                                ),
                                "use_device_time": ParameterValue(
                                    use_device_time, value_type=bool
                                ),
                                "publisher_queue_depth": ParameterValue(
                                    publisher_queue_depth, value_type=int
                                ),
                            }
                        ],
                    )
                ],
            ),
        ]
    )
