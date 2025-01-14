# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

"""Launch realsense2_camera node."""
import copy
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch
from launch.substitutions import LaunchConfiguration

package_name='ros_phoenix'

local_parameters = [{'name': 'camera_name1', 'default': 'depth_camera_imu', 'description': 'camera unique name'},
                    {'name': 'camera_name2', 'default': 'depth_camera', 'description': 'camera unique name'},
                    {'name': 'usb_port_id1', 'default': "''", 'description': 'choose device by serial number'},
                    {'name': 'usb_port_id2', 'default': "''", 'description': 'choose device by serial number'},
                    {'name': 'device_type1', 'default': "'d435i'", 'description': 'choose device by type'},
                    {'name': 'device_type2', 'default': "'d435'", 'description': 'choose device by type'},
                    {'name': 'enable_gyro1', 'default': 'true', 'description': "''"},                           
                    {'name': 'enable_accel1','default': 'true', 'description': "''"},
                    {'name': 'pointcloud.enable1', 'default': 'true', 'description': ''},
                    {'name': 'pointcloud.enable2', 'default': 'false', 'description': ''}, 
                    {'name': 'unite_imu_method1', 'default': "2", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                    {'name': 'unite_imu_method2', 'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                    {'name': 'publish_tf1', 'default': 'false', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                    {'name': 'tf_publish_rate1', 'default': '0.0', 'description': '[double] rate in Hz for publishing dynamic TF'},
                    {'name': 'enable_sync1',                  'default': 'false', 'description': "''"},                           


                   ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params
    
def add_node_action(context : LaunchContext):
    # dummy static transformation from camera1 to camera2
    node = launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0",
                          context.launch_configurations['camera_name1'] + "_link",
                          context.launch_configurations['camera_name2'] + "_link"]
    )
    return [node]

# def generate_launch_description():
#     params1 = duplicate_params(rs_launch.configurable_parameters, '1')
#     params2 = duplicate_params(rs_launch.configurable_parameters, '2')
#     return LaunchDescription(
#         rs_launch.declare_configurable_parameters(local_parameters) +
#         rs_launch.declare_configurable_parameters(params1) + 
#         rs_launch.declare_configurable_parameters(params2) + 
#         [
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
#             launch_arguments=set_configurable_parameters(params1).items(),
#         ),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
#             launch_arguments=set_configurable_parameters(params2).items(),
#         ),
#         OpaqueFunction(function=add_node_action)
#     ])
def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
        ),
        OpaqueFunction(function=add_node_action)
    ])
