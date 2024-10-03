
# Este codigo se encarga de lanzar TODOS los codigos necesarios para correr la simulacion, exceptuando el de la comunicacion con arduino.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='scara_joints',
            executable='interfaz_usuario.py',
            name='interfaz_usuario_joints',
        ),
        Node(
            package='scara_joints',
            executable='joint_publisher.py',
            name='joint_publisher_node',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('scara_urdf'),
                    'launch',
                    'display.launch.py'
                ])
            ),
        )
    ])
