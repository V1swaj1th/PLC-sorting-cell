from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' name:=ur ur_type:=ur5e']), value_type=str)
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('sorting_cell_bringup'), 'launch', 'teach.rviz'])

    # Stand-alone teach mode. Do not run it together with sorting.launch.py: both publish joint_states.
    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='sorting_cell_controller', executable='teach_pendant', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
