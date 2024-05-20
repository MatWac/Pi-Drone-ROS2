import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    pkg_path = os.path.join(get_package_share_directory('icm_publisher'))
    config_file_path = os.path.join(pkg_path, 'config/ekf.yaml')
    
    icm_publisher = Node(
        package="icm_publisher",
        executable="icm_publisher",
        parameters=[{
            'frame_id': 'base_footprint'
        }]
    )
    
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[config_file_path]
    )
    
    imu_filter_madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'gain': 0.6
        }],
        output='screen'
    )
    
    imu_complementary_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter',
        parameters=[{
            'bias_alpha': 0.01,
            'gain_acc': 0.9   
        }],
        output='screen'
    )
    
    return LaunchDescription([
        ekf,
        imu_filter_madgwick,
        #imu_complementary_filter,
        icm_publisher      
    ])