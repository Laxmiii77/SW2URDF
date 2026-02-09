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

using SW2URDF.UI;
using System;
using System.IO;

namespace SW2URDF.URDFExport
{
    public class URDFPackage
    {
        public static IMessageBox MessageBox = new MessageBoxHelper();
        public string PackageName { get; }

        public string PackageDirectory { get; }
        public string MeshesDirectory { get; }
        public string TexturesDirectory { get; }
        public string RobotsDirectory { get; }
        public string ConfigDirectory { get; }
        public string LaunchDirectory { get; }

        public string WindowsPackageDirectory { get; }
        public string WindowsMeshesDirectory { get; }
        public string WindowsTexturesDirectory { get; }
        public string WindowsRobotsDirectory { get; }
        public string WindowsLaunchDirectory { get; }
        public string WindowsConfigDirectory { get; }
        public string WindowsWorldsDirectory { get; }
        public string WindowsCMakeLists { get; }
        public string WindowsConfigYAML { get; }
        public string WindowsGzBridgeYAML { get; }
        public string WindowsEmptyWorldSDF { get; }

        public URDFPackage(string name, string dir)
        {
            PackageName = name;
            PackageDirectory = @"package://" + name + @"/";
            MeshesDirectory = PackageDirectory + @"meshes/";
            RobotsDirectory = PackageDirectory + @"urdf/";
            TexturesDirectory = PackageDirectory + @"textures/";
            LaunchDirectory = PackageDirectory + @"launch/";
            ConfigDirectory = PackageDirectory + @"config/";


            char last = dir[dir.Length - 1];
            dir = (last == '\\') ? dir : dir + @"\";
            WindowsPackageDirectory = dir + name + @"\";
            WindowsMeshesDirectory = WindowsPackageDirectory + @"meshes\";
            WindowsRobotsDirectory = WindowsPackageDirectory + @"urdf\";
            WindowsTexturesDirectory = WindowsPackageDirectory + @"textures\";
            WindowsLaunchDirectory = WindowsPackageDirectory + @"launch\";
            WindowsConfigDirectory = WindowsPackageDirectory + @"config\";
            WindowsWorldsDirectory = WindowsPackageDirectory + @"worlds\";
            WindowsCMakeLists = WindowsPackageDirectory + @"CMakeLists.txt";
            WindowsConfigYAML = WindowsConfigDirectory + @"joint_names_" + name + ".yaml";
            WindowsGzBridgeYAML = WindowsConfigDirectory + @"gz_bridge.yaml";
            WindowsEmptyWorldSDF = WindowsWorldsDirectory + @"empty_world.sdf";
        }

        public void CreateDirectories()
        {
            MessageBox.Show("Creating URDF Package \"" +
                PackageName + "\" at:\n" + WindowsPackageDirectory);
            if (!Directory.Exists(WindowsPackageDirectory))
            {
                Directory.CreateDirectory(WindowsPackageDirectory);
            }
            if (!Directory.Exists(WindowsMeshesDirectory))
            {
                Directory.CreateDirectory(WindowsMeshesDirectory);
            }
            if (!Directory.Exists(WindowsRobotsDirectory))
            {
                Directory.CreateDirectory(WindowsRobotsDirectory);
            }
            if (!Directory.Exists(WindowsTexturesDirectory))
            {
                Directory.CreateDirectory(WindowsTexturesDirectory);
            }
            if (!Directory.Exists(WindowsLaunchDirectory))
            {
                Directory.CreateDirectory(WindowsLaunchDirectory);
            }
            if (!Directory.Exists(WindowsConfigDirectory))
            {
                Directory.CreateDirectory(WindowsConfigDirectory);
            }
            if (!Directory.Exists(WindowsWorldsDirectory))
            {
                Directory.CreateDirectory(WindowsWorldsDirectory);
            }
            CreateEmptyWorldSDF();
            CreateGzBridgeYAML();
        }

        public void CreateCMakeLists()
        {
            using (StreamWriter file = new StreamWriter(WindowsCMakeLists))
            {
                file.WriteLine("cmake_minimum_required(VERSION 3.8)\r\n");
                file.WriteLine("project(" + PackageName + ")\r\n");
                file.WriteLine("find_package(ament_cmake REQUIRED)\r\n");
                file.WriteLine("install(DIRECTORY config launch meshes urdf worlds/");
                file.WriteLine("\tDESTINATION share/${PROJECT_NAME})");
                file.WriteLine("ament_package()");
            }
        }

        public void CreateConfigYAML(String[] jointNames)
        {
            using (StreamWriter file = new StreamWriter(WindowsConfigYAML))
            {
                file.Write("controller_joint_names: " + "[");

                foreach (String name in jointNames)
                {
                    file.Write("'" + name + "', ");
                }

                file.WriteLine("]");
            }
        }

        public void CreateGzBridgeYAML()
        {
            string gzBridgeContent = @"
- ros_topic_name: ""clock""
  gz_topic_name: ""clock""
  ros_type_name: ""rosgraph_msgs/msg/Clock""
  gz_type_name: ""gz.msgs.Clock""
  direction: GZ_TO_ROS

- ros_topic_name: ""joint_states""
  gz_topic_name: ""joint_states""
  ros_type_name: ""sensor_msgs/msg/JointState""
  gz_type_name: ""gz.msgs.Model""
  direction: GZ_TO_ROS

- ros_topic_name: ""tf""
  gz_topic_name: ""tf""
  ros_type_name: ""tf2_msgs/msg/TFMessage""
  gz_type_name: ""gz.msgs.Pose_V""
  direction: GZ_TO_ROS
";

            File.WriteAllText(WindowsGzBridgeYAML, gzBridgeContent);
        }

        public void CreateEmptyWorldSDF()
        {
            string sdfContent = @"<sdf version='1.10'>
  <world name='empty'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
    <gravity>0 0 -9.8000000000000007</gravity>
    <magnetic_field>5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>0.400000006 0.400000006 0.400000006 1</ambient>
      <background>0.699999988 0.699999988 0.699999988 1</background>
      <shadows>true</shadows>
    </scene>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.800000012 0.800000012 0.800000012 1</ambient>
            <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
            <specular>0.800000012 0.800000012 0.800000012 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name='sun' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.10000000000000001 -0.90000000000000002</direction>
      <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
      <specular>0.200000003 0.200000003 0.200000003 1</specular>
      <attenuation>
        <range>1000</range>
        <linear>0.0099999997764825821</linear>
        <constant>0.89999997615814209</constant>
        <quadratic>0.0010000000474974513</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
  </world>
</sdf>";

            File.WriteAllText(WindowsEmptyWorldSDF, sdfContent);
        }
    }
}