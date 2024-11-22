
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    launch_description = LaunchDescription()

    # get pkg path
    multi_robots_pkg_path = get_package_share_directory('nav2_multi_robots_avoidance')    

    # get param file path
    params_file_path = os.path.join(multi_robots_pkg_path, 'param', 'config.yaml')   

    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level", default='info')

    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='define multi_robots_avoidance node log level')

    priority = 1
    try:
        if 'ROBOT_PRIORITY' in os.environ:
            priority = int(os.environ.get('ROBOT_PRIORITY'))
            print(f'get ROBOT_PRIORITY value {priority} from os.environment.')
        else:
            print('using default ROBOT_PRIORITY value 1')
            priority = 1
    except:
        print("Please declare ROBOT_PRIORITY!")
        priority = 1

    # multi_robots_avoidance Node
    nav2_multi_robots_avoidance_node = Node(
        executable='multi_robots_avoidance',
        package='nav2_multi_robots_avoidance',
        name='multi_robots_avoidance',
        namespace=namespace,
        output='screen',
        parameters=[params_file_path, {"use_sim_time": False, "priority": priority}],
        remappings=[("/tf", "tf"),
                    ("/tf_static", "tf_static")],
        arguments=['--ros-args','--log-level', ['multi_robots_avoidance:=', LaunchConfiguration('log_level')]],
    )

    launch_description.add_action(log_level_arg)
    launch_description.add_action(nav2_multi_robots_avoidance_node)

    return launch_description





