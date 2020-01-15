# Copyright 2019 Intelligent Robotics Lab
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
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

  # Create the launch configuration variables
  context_file = LaunchConfiguration('context_file')
  params_file = LaunchConfiguration('params_file')
  namespace = LaunchConfiguration('namespace')
  
  df_dir = get_package_share_directory('dialogflow_ros_launch')

# Create our own temporary YAML files that include substitutions
  param_substitutions = {}

  configured_params = RewrittenYaml(
    source_file=params_file,
    root_key=namespace,
    param_rewrites=param_substitutions,
    convert_types=True)

  declare_namespace_cmd = DeclareLaunchArgument(
    'namespace', default_value='',
    description='Top-level namespace')

  declare_params_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(df_dir, 'config', 'params.yaml'),
    description='Full path to the DialogFlow parameters file to use')
  
  declare_context_file_cmd = DeclareLaunchArgument(
    'context_file',
    default_value=os.path.join(df_dir, 'config', 'context.yaml'),
    description='DialogFlow contexts')

  # Specify the actions
  df_client_cmd = Node(
    package='dialogflow_ros',
    node_executable='dialogflow_client',
    node_name='dialogflow_client',
    output='screen',
    parameters=[
      configured_params,
      {'context_file': context_file}])

  # Create the launch description and populate
  ld = LaunchDescription()

  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_params_cmd)
  ld.add_action(declare_context_file_cmd)

  # Declare the launch options
  ld.add_action(df_client_cmd)

  return ld