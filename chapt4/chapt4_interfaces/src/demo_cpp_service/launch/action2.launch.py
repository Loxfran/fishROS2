import launch
import launch_ros
from launch.conditions import IfCondition


def generate_launch_description():
    # 声明参数，是否创建海龟
    declare_spawn_turtle = launch.actions.DeclareLaunchArgument(
        'spawn_turtle',
        default_value='true',
        description='Whether to spawn a second turtle')
    spawn_turtle = launch.substitutions.LaunchConfiguration('spawn_turtle')
    action_turtlesim = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen',
    )
    # 给日志输出和服务调用添加条件
    action_execute_process = launch.actions.ExecuteProcess(
        condition=IfCondition(spawn_turtle),
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'],
    )
    action_log_info = launch.actions.LogInfo(
        condition=IfCondition(spawn_turtle),
        msg='Turtlesim node has been launched and turtle2 has been spawned.'
    )
    # 利用TimerAction延时启动，利用action group组合多个action
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
        launch.actions.TimerAction(period=3.0, actions=[action_execute_process]),
    ])
    # 返回启动描述
    return launch.LaunchDescription([
        declare_spawn_turtle,
        action_turtlesim,
        action_group
        ])