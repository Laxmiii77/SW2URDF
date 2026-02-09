/*
Copyright (c) 2015 Stephen Brawner
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
using System.IO;
using System;
namespace SW2URDF.ROS
{
    public class Gazebo
    {
        private readonly string package;
        private readonly string robotURDF;
        private readonly string model;
        public Gazebo(string modelName, string packageName, string URDFName)
        {
            model = modelName;
            package = packageName;
            robotURDF = URDFName;
        }
        public void WriteFile(string dir)
        {
            string path = dir + @"gazebo.launch.py";
            string content = string.Format(
@"import os 
from launch_ros.actions import Node 
from launch import LaunchDescription 
from launch.conditions import IfCondition 
from ament_index_python.packages import get_package_share_directory 
from launch.substitutions import LaunchConfiguration, PythonExpression 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess 
import tempfile
import re

def resolve_package_urls(urdf_content):
    """"""
    Replace all package:// URIs with absolute file:// paths
    """"""
    def replace_package_url(match):
        package_name = match.group(1)
        path = match.group(2)
        try:
            package_path = get_package_share_directory(package_name)
            return f'file://{{os.path.join(package_path, path)}}'
        except:
            return match.group(0)  # Return original if package not found
    
    # Pattern to match package://package_name/path
    pattern = r'package://([^/]+)/(.+?)(?=[""\\s<>])'
    return re.sub(pattern, replace_package_url, urdf_content)

def generate_launch_description(): 
    # Package name 
    package_name = '{0}' 
    
    # Get package directory
    pkg_share = get_package_share_directory('{0}')
    
    # Launch configurations 
    world = LaunchConfiguration('world') 
    
    # Path to default world  
    world_path = os.path.join(pkg_share, 'empty_world.sdf')
    
    # Path to URDF file
    default_urdf_path = os.path.join(pkg_share, ""urdf"", '{0}.urdf')
    
    # Read and process URDF
    with open(default_urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Resolve all package:// URIs to absolute paths
    urdf_content = resolve_package_urls(urdf_content)
    
    # Write to temporary file
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    temp_urdf.write(urdf_content)
    temp_urdf.close()
    processed_urdf_path = temp_urdf.name
    
    # Launch Arguments 
    declare_world = DeclareLaunchArgument( 
        name='world', 
        default_value=world_path, 
        description='Full path to the world model file to load'
    ) 
    
    declare_rviz = DeclareLaunchArgument( 
        name='rviz', 
        default_value='True', 
        description='Opens rviz if set to True'
    ) 
    
    urdf_path_arg = DeclareLaunchArgument(
        name='urdf_path',
        default_value=default_urdf_path
    )
    
    urdf_path = LaunchConfiguration('urdf_path')
    
    # Launch the gazebo server to initialize the simulation 
    gazebo_server = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py' 
        )]), 
        launch_arguments={{'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}}.items() 
    ) 
    
    # Launch the gazebo client to visualize the simulation 
    gazebo_client = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py' 
        )]), 
        launch_arguments={{'gz_args': '-g'}}.items() 
    ) 
    
    # Run the spawner node from ros_gz_sim package  
    spawn_entity = ExecuteProcess( 
        cmd=[ 
            'ros2', 'run', 'ros_gz_sim', 'create', 
            '-name', 'bot', 
            '-file', processed_urdf_path,
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.2' 
        ], 
        output='screen' 
    ) 
    
    # Launch the Gazebo-ROS bridge 
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node( 
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        parameters=[{{'config_file': bridge_params}}]
    ) 
    
    # Launch them all! 
    return LaunchDescription([ 
        # Declare launch arguments 
        declare_rviz, 
        declare_world, 
        urdf_path_arg,
        # Launch the nodes 
        gazebo_server, 
        gazebo_client, 
        ros_gz_bridge, 
        spawn_entity, 
    ])", package);
            File.WriteAllText(path, content);
        }
    }
    public class Rviz
    {
        private readonly string package;
        private readonly string robotURDF;
        public Rviz(string packageName, string URDFName)
        {
            package = packageName;
            robotURDF = URDFName;
        }
        public void WriteFiles(string dir)
        {
            string path = dir + @"display.launch.py";
            string content = string.Format(
@"import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    {0}_pkg = get_package_share_directory(""{0}"")
    default_urdf_path = os.path.join({0}_pkg, ""urdf"", ""{0}.urdf"")
    urdf_path_arg = DeclareLaunchArgument(
        name=""urdf_path"",
        default_value=default_urdf_path
    )
    urdf_path = LaunchConfiguration(""urdf_path"")
    robot_description_content = open(default_urdf_path).read()
    robot_state_publisher = Node(
        package=""robot_state_publisher"",
        executable=""robot_state_publisher"",
        parameters=[{{""robot_description"": robot_description_content}}],
    )
    joint_state_publisher = Node(
        package=""joint_state_publisher"",
        executable=""joint_state_publisher"",
    )
    rviz_node = Node(
        package=""rviz2"",
        executable=""rviz2"",
        name=""rviz2""
    )
    return LaunchDescription([
        urdf_path_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])", package);
            File.WriteAllText(path, content);
        }
    }
}