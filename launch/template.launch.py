
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mess2_logger_cpp_share = FindPackageShare(package="mess2_logger_cpp").find("mess2_logger_cpp")
    mess2_logger_cpp_parameters = mess2_logger_cpp_share + "/config/template.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "file_config", 
            default_value=mess2_logger_cpp_parameters, 
            description="path to the configuration file"),
        DeclareLaunchArgument(
            name="dir_logs",
            default_value="Desktop/logs",
            description="relative path from home to the location at which subdirs will be created for each camera"
        ),
        Node(
            package="mess2_logger_cpp",
            executable="log_topics_to_jpgs",
            name="log_topics_to_jpgs",
            output="screen",
            parameters=[LaunchConfiguration("file_config")]
        ),
    ])
