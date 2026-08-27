import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('chassis_driver'),
        'config',
        'chassis_driver_params.yaml'
    )

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # 駆動ノードの作成
    chassis_driver_node = Node(
        package = 'chassis_driver',
        executable = 'chassis_driver',
        parameters = [config_file_path],
        output='screen'
    )
    # socketcanノードの作成
    socketcan_node = Node(
        package = 'socketcan_interface',
        executable = 'socketcan_interface_node',
        parameters = [config_file_path],
        output='screen'
    )
    # joyノードの作成
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [config_file_path],
        output='screen'
    )
    # odrive起動の作成
    odrive_launch = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('chassis_driver'), 'launch/'),
            'odrive_can_launch.yaml'])
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['odrive'] is True):
        launch_discription.add_action(odrive_launch)
    if(launch_params['socketcan'] is True):
        launch_discription.add_action(socketcan_node)

    launch_discription.add_action(joy_node)
    launch_discription.add_action(chassis_driver_node)

    return launch_discription
