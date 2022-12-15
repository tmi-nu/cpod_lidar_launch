import os
import yaml

import ament_index_python.packages
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
 
    cpod_lidar_launch_dir = ament_index_python.packages.get_package_share_directory('cpod_lidar_launch')
    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')


    # Front Velodyne
    driver_params_file_front = os.path.join(cpod_lidar_launch_dir, 'config', 'front-lidar-params.yaml')


    ## Velodyne Driver 
    front_velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="front_velodyne_driver_node",
       #  parameters=[{
       #      "device_ip": "192.168.0.201",
       #      "port": 2368,
       #      }],
        parameters=[driver_params_file_front],
        remappings=[
            ("velodyne_packets", "front_velodyne_packets")
        ]
    )

    ## Converter Node
    convert_params_file_front = os.path.join(cpod_lidar_launch_dir, 'config', 'front_lidar_convert_node-params.yaml')
    with open(convert_params_file_front, 'r') as f:
        convert_params_front = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params_front['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    front_velodyne_convert_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_convert_node",
        name="front_velodyne_convert_node",
        parameters=[convert_params_front],
        remappings=[
            ("velodyne_packets", "front_velodyne_packets"),
            ("velodyne_points", "front_velodyne_points")
        ]
    )


    ## publish tf for front velodyne
    static_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_front",
        output="screen",
        arguments=["0.345","0", "0", "0", "0","1","0","lidar","front"],
    )


    # Back Velodyne
    driver_params_file_back = os.path.join(cpod_lidar_launch_dir, 'config', 'back-lidar-params.yaml')


    ## Velodyne driver
    back_velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="back_velodyne_driver_node",
       #  parameters=[{
       #      "device_ip": "192.168.0.202",
       #      "port": 2369,
       #      }],
        parameters=[driver_params_file_back],
        remappings=[
            ("velodyne_packets", "velodyne_packets_2")
        ]
    )

    ## Converter Node
    convert_params_file_back = os.path.join(cpod_lidar_launch_dir, 'config', 'back_lidar_convert_node-params.yaml')
    with open(convert_params_file_back, 'r') as f:
        convert_params_back = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params_back['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    back_velodyne_convert_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_convert_node",
        name="back_velodyne_convert_node",
        parameters=[convert_params_back],
        remappings=[
            ("velodyne_packets", "back_velodyne_packets"),
            ("velodyne_points", "back_velodyne_points")
        ]
    )

    ## publish tf for back velodyne
    static_back = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_back",
        output="screen",
        arguments=["-0.345","0.3", "0", "0", "0","0","1","lidar","back"],
    )
    ld.add_action(front_velodyne_driver_node)
    ld.add_action(front_velodyne_convert_node)
    ld.add_action(back_velodyne_driver_node)
    ld.add_action(back_velodyne_convert_node)
    ld.add_action(static_front)
    ld.add_action(static_back)
    return ld
