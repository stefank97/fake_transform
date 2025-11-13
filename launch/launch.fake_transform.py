from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration , PythonExpression
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('fake_transform'),
        'config',
        'params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation clock if true')

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='fake_transform',
            executable='fake_transform',
            name='fake_transform_node',
            output='screen',
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
            arguments=['--ros-args','--log-level','info']
        )
    ])