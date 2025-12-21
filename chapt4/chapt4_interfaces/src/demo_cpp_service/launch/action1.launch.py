import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 利用IncludeLaunchDescription包含另一个launch文件
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('turtlesim'), '/launch', '/multisim.launch.py']
        )
    )
    # 利用ExecuteProcess启动
    action_execute_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/turtlesim1/spawn', 'turtlesim/srv/Spawn', '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'],
    )
    # 利用log输出日志
    action_log_info = launch.actions.LogInfo(
        msg='Turtlesim nodes have been launched and turtle2 has been spawned.'
    )
    # 利用TimerAction延时启动，利用action group组合多个action
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
        launch.actions.TimerAction(period=3.0, actions=[action_execute_process]),
    ])
    # 返回启动描述
    return launch.LaunchDescription([
        action_include_launch,
        action_group])