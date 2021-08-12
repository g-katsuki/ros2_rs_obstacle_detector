from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  pkg_prefix = get_package_share_directory('ros2_rs_obstacle_detector')
  container = Node(
    package='rclcpp_components',
    executable='component_container',
    name='local_path_planner_container2'
  )
  components = LoadComposableNodes(
    target_container = 'local_path_planner_container2',
    composable_node_descriptions = [
      ComposableNode(
        package = 'ros2_rs_obstacle_detector',
        plugin  = 'project_ryusei::ROS2ObstacleDetector',
        name    = 'rs_obstacle_detector',
        extra_arguments = [
          {'use_intra_process_comms': True}
        ],
        parameters=[
          join(pkg_prefix, 'cfg/obstacle_detector.yaml')
          # {'resource_directory_name': 'cfg/navigation_cfg'},
          # {'init_file_name': 'navigation.ini'},
          # {'pose_name': '/pose'},
          # {'locator_name': '/locator'}
        ]
      )
    ]
  )
  return LaunchDescription([
    container,
    components
  ])