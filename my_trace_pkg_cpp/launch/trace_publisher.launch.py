import launch

import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_trace_pkg_cpp',
            executable='trace_publisher',
            name='trace_publisher_node'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()