from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import pi

_90_deg_ccw = str(pi/2)
_90_deg_cw = str(-pi/2)
_180_deg = str(pi)
_45_deg_up = str(-pi/4)

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if true'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '-0.096', '--z', '0.258', '--yaw', _90_deg_cw, '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'ti_mmwave_0'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.096', '--y', '0', '--z', '0.258', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'ti_mmwave_1'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '-0.115', '--y', '0', '--z', '0.258', '--yaw', _180_deg, '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'ti_mmwave_2'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0.096', '--z', '0.258', '--yaw', _90_deg_ccw, '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'ti_mmwave_3'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.080', '--y', '-0.018', '--z', '0.330', '--yaw', '0', '--pitch', _45_deg_up, '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'ti_mmwave_4'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '-0.080', '--y', '0.018', '--z', '0.330', '--yaw', _180_deg, '--pitch', _45_deg_up, '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'ti_mmwave_5'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0.216', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.255', '--y', '0', '--z', '-0.070', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'radar'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0.136', '--yaw', _90_deg_cw, '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'vectornav'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'dlio/odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
])
