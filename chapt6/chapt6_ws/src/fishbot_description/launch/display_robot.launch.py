import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取urdf文件路径
    fishbot_description_path = get_package_share_directory('fishbot_description')
    urdf_file_path = os.path.join(fishbot_description_path, 'urdf', 'first_robot.xacro')
    default_rviz_config_path = os.path.join(fishbot_description_path, 'config', 'display_robot_model.rviz')
    # 声明一个urdf目录的参数，方便修改
    action_declare_urdf_path = launch.actions.DeclareLaunchArgument(
        name='urdf_path',
        default_value=str(urdf_file_path),
        description= '加载的模型文件路径'
    )
    # 通过文件路径，获取内容，转换成参数值对象，以供传入robot_state_publisher节点
    substitutions_command_result = launch.substitutions.Command(
        command=['xacro ', launch.substitutions.LaunchConfiguration('urdf_path')]
    )
    robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
        value=substitutions_command_result,
        value_type=str)

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot_state_publisher',
        # output='screen',
        parameters=[{
            'robot_description': robot_description_content
            }]
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher')

    action_rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
        )


    return launch.LaunchDescription([
        action_declare_urdf_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz2,
    ])