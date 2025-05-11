import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Path to the configuration file for lunahcam
    config = os.path.join(
        get_package_share_directory('lunahcam'),
        'config',
        'config.yaml'
    )

    # Environment variable for DDS selection (choose a DDS implementation that is known for RT performance)

    # Prefix for real-time scheduling; using sudo -E to preserve environment variables

    bus_interface = Node(
        package='lunahcam',
        executable='bus_interface',
        name='bus_interface',
        parameters=[config]
    )

    cmd_handler = Node(
        package='lunahcam',
        executable='cmd_handler',
        name='cmd_handler',
        parameters=[config]
    )

    pre_shutdown = Node(
        package='lunahcam',
        executable='pre_shutdown',
        name='pre_shutdown',
        parameters=[config]
    )

    payload = Node(
        package='lunahcam',
        executable='payload',
        name='payload',
        parameters=[config]
    )

    thermal_control = Node(
        package='lunahcam',
        executable='thermal_control',
        name='thermal_control',
        parameters=[config]
    )

    telemetry_data = Node(
        package='lunahcam',
        executable='telemetry_data',
        name='telemetry_data',
        parameters=[config]
    )

    ld.add_action(bus_interface)
    #ld.add_action(telemetry_data)
    ld.add_action(cmd_handler)
    ld.add_action(payload)
    #ld.add_action(thermal_control)
    #ld.add_action(pre_shutdown)

    return ld
