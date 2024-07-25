from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('ForROS_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'ForROS.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    gnss_node = Node(
        package='GNSS',
        executable='gnss_node',
        name='gnss',
        namespace='gnss',                 
        #remappings=[('chatter', 'chatter_app1')]    
    )
    bme280_node = Node(
        package='bme280',
        executable='bme280_node',
        name='bme280',
        namespace='bme280',                 
        #remappings=[('chatter', 'chatter_app1')]    
    )
    bno055_node = Node(
        package='bno055',
        executable='bno055_node',
        name='bno055',
        namespace='bno055',                 
        #remappings=[('chatter', 'chatter_app1')]    
    )
    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera',
        namespace='camera',                 
        #remappings=[('chatter', 'chatter_app1')]    
    )
    yolov10_node = Node(
        package='yolov10',
        executable='yolov10_node',
        name='yolov10',
        namespace='yolov10',                 
        #remappings=[('chatter', 'chatter_app1')]    
    )
    motor_node = Node(
        package='motor',
        executable='motor_node',
        name='motor',
        namespace='motor',                 
        #remappings=[('chatter', 'chatter_app1')]    
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        gnss_node,
        bme280_node,
        bno055_node,
        camera_node,
        yolov10_node,
        motor_node,
        #joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        rviz_node
    ])
