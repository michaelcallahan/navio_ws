from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_node',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='gps_node',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),
        Node(
            package='battery_monitor_node',
            executable='battery_monitor_node',
            name='battery_monitor_node',
            output='screen'
        ),
        Node(
            package='state_estimator_node',
            executable='state_estimator_node',
            name='state_estimator_node',
            output='screen'
        ),
        Node(
            package='controller_node',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='mission_planner_node',
            executable='mission_planner_node',
            name='mission_planner_node',
            output='screen'
        ),
        Node(
            package='motor_control_node',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'
        )
    ])
