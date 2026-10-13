from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_plc = LaunchConfiguration('use_plc')
    drive = LaunchConfiguration('drive')
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' name:=ur ur_type:=ur5e']), value_type=str)
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('sorting_cell_bringup'), 'launch', 'sorting_cell.rviz'])
    default_scene = PathJoinSubstitution(
        [FindPackageShare('sorting_cell_bringup'), 'launch', 'scene_default.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_plc', default_value='false',
                              description='true: PLC (Modbus) gates the motion; false: stand-alone'),
        DeclareLaunchArgument('plc_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('plc_port', default_value='5020'),
        DeclareLaunchArgument('scene', default_value=default_scene,
                              description='scene YAML: objects, coloured containers, gray container'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='stand-alone only: true starts by itself after 5 s; '
                                          'false waits for Start in the dashboard (cell/start)'),
        DeclareLaunchArgument('drive', default_value='true',
                              description='true: servo drive + simulated joints between the planner and '
                                          'joint_states; false: the planner sets the joint angles directly'),
        DeclareLaunchArgument('cube_mass_kg', default_value='0.5',
                              description='mass the drive feels while a cube is carried'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='serve the scene dashboard in the browser'),
        DeclareLaunchArgument('dashboard_host', default_value='127.0.0.1',
                              description='127.0.0.1: this machine only; 0.0.0.0: reachable from other hosts'),
        DeclareLaunchArgument('dashboard_port', default_value='8765'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='sorting_cell_controller', executable='scene_server',
             parameters=[{'scene_file': LaunchConfiguration('scene')}], output='screen'),
        Node(package='sorting_cell_vision', executable='vision_node',
             parameters=[{'publish_markers': False}], output='screen'),
        Node(package='sorting_cell_controller', executable='sorting_sequencer',
             parameters=[{'start_delay_s': 5.0,
                          'use_plc': ParameterValue(use_plc, value_type=bool),
                          'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                          'use_drive': ParameterValue(drive, value_type=bool),
                          'cube_mass_kg': ParameterValue(LaunchConfiguration('cube_mass_kg'), value_type=float)}],
             output='screen'),
        Node(package='sorting_cell_controller', executable='joint_drive',
             parameters=[{'use_plc': ParameterValue(use_plc, value_type=bool)}],
             condition=IfCondition(drive), output='screen'),
        Node(package='sorting_cell_controller', executable='plc_bridge',
             parameters=[{'host': LaunchConfiguration('plc_host'),
                          'port': ParameterValue(LaunchConfiguration('plc_port'), value_type=int)}],
             condition=IfCondition(use_plc), output='screen'),
        Node(package='sorting_cell_dashboard', executable='dashboard_node',
             parameters=[{'host': LaunchConfiguration('dashboard_host'),
                          'port': ParameterValue(LaunchConfiguration('dashboard_port'), value_type=int),
                          'use_plc': ParameterValue(use_plc, value_type=bool),
                          'drive': ParameterValue(drive, value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('dashboard')), output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
