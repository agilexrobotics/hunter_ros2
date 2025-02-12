import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
                                         description='CAN bus name, e.g. can0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='/diff_drive_controller/odom',
                                           description='Odometry topic name')

    simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
                                                   description='Whether running with simulator')
    sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')

    enable_pd_regulator = LaunchConfiguration('enable_pd_regulator', default='False')
    
    kp_v = LaunchConfiguration('kp_v', default='40.0')
    kd_v = LaunchConfiguration('kd_v', default='0.1') 
    kp_w = LaunchConfiguration('kp_w', default='12.0')
    kd_w = LaunchConfiguration("kd_w", default="0.1")
    
    kp_v_val = DeclareLaunchArgument('kp_v', default_value=kp_v, description='Proportional gain for linear velocity')
    kd_v_val = DeclareLaunchArgument('kd_v', default_value=kd_v, description='Derivative gain for linear velocity')
    kp_w_val = DeclareLaunchArgument('kp_w', default_value=kp_w, description='Proportional gain for angular velocity')
    kd_w_val = DeclareLaunchArgument('kd_w', default_value=kd_w, description='Derivative gain for angular velocity')
    
    enable_pd_regulator_val = DeclareLaunchArgument('enable_pd_regulator'
        , default_value=enable_pd_regulator
        , description='Use PD regulator estimate residual control to the robot')
    
    hunter_base_node = launch_ros.actions.Node(
        package='hunter_base',
        executable='hunter_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
                'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
                'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
                'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
                'simulated_robot': launch.substitutions.LaunchConfiguration('simulated_robot'),
                'control_rate': launch.substitutions.LaunchConfiguration('control_rate'),
                'kp_v': kp_v,
                'kd_v': kd_v,
                'kp_w': kp_w,
                'kd_w': kd_w,
                'enable_pd_regulator': enable_pd_regulator
        }],
        remappings=[
            ("/hunter/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
            ("/hunter/global_odom", "/odometry/global")
        ])

    return LaunchDescription([
        use_sim_time_arg,
        port_name_arg,        
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        simulated_robot_arg,
        sim_control_rate_arg,
        hunter_base_node,
        enable_pd_regulator_val,
        kp_v_val,
        kd_v_val,
        kp_w_val,
        kd_w_val
    ])
