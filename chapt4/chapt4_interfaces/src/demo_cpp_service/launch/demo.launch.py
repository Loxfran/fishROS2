import launch
import launch_ros


def generate_launch_description():
    # 创建声明参数声明
    action_declare_arg_max_speed = launch.actions.DeclareLaunchArgument(
        'launch_max_speed',
        default_value='2.0')
    action_declare_arg_k = launch.actions.DeclareLaunchArgument(
        'launch_k',
        default_value='1.0')
    # 1.启动turtle_controller节点
    turtle_controller_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        parameters=[
            {'max_speed': launch.substitutions.LaunchConfiguration('launch_max_speed'),
             'k': launch.substitutions.LaunchConfiguration('launch_k')}
        ],
        output='screen',
    )
    # 2.启动patrol_client节点
    action_client_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        output='log',
    )
    # 3.启动turtlesim节点
    turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='both',
    )
    # 返回启动描述
    return launch.LaunchDescription([
        action_declare_arg_max_speed,
        action_declare_arg_k,
        turtle_controller_node,
        action_client_node,
        turtlesim_node
    ])


