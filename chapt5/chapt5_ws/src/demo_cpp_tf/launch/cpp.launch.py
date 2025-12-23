import launch
import launch_ros

def generate_launch_description():
    static_broadcaster = launch_ros.actions.Node(
        package='demo_cpp_tf',
        executable='static_tf_broadcaster',
        output='screen',
    )
    dynamic_broadcaster = launch_ros.actions.Node(
        package='demo_cpp_tf',
        executable='dynamic_tf_broadcaster',
        output='screen',
    )
    tf_listener = launch_ros.actions.Node(
        package='demo_cpp_tf',
        executable='tf_listener',
        output='screen',
    )

    return launch.LaunchDescription([
        static_broadcaster,
        dynamic_broadcaster,
        # tf_listener,
    ])
