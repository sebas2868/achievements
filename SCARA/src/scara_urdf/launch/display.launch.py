
# Este codigo se encarga de iniciar RVIZ, junto con los nodos y topicos nativos de ROS cuya funcion final es visualizar un robot descrito por archivos URDF.

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file_name = 'scara.urdf'
    urdf = os.path.join(
        get_package_share_directory('scara_urdf'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
        #                                      description='Flag to enable joint_state_publisher_gui'),

        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])