import launch
import launch_ros

def generate_launch_description():
    # 1.启动face_detect_node节点
    face_detect_node = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_node',
        output='screen',
    )
    # 2.启动face_detect_client节点
    face_detect_client_node = launch_ros.actions.Node(
        package='demo_python_service',
        executable='face_detect_client',
        output='screen',
    )
    # 返回启动描述
    return launch.LaunchDescription([
        face_detect_node,
        face_detect_client_node
    ])


