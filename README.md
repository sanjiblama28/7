# Managing large projets

## 1. Top-level organization

We will first construct a launch file that will call other launch files. Create a launch_turtlesim.launch.py file in the launch_tutorial package's /launch folder to accomplish this.

```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_1.launch.py'])
      )
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
   mimic_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/mimic.launch.py'])
      )
   fixed_frame_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/fixed_broadcaster.launch.py'])
      )
   rviz_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_rviz.launch.py'])
      )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2,
      broadcaster_listener_nodes,
      mimic_node,
      fixed_frame_node,
      rviz_node
   ])
```

## 2. Parameters
## 2.1 Setting parameters in the launch file

The first thing we'll do is create the launch file that will launch our first turtlesim simulation. In the beginning, make a new file called turtlesim_world_1.launch.py.

```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')
   )

   return LaunchDescription([
      background_r_launch_arg,
      background_g_launch_arg,
      background_b_launch_arg,
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         parameters=[{
            'background_r': LaunchConfiguration('background_r'),
            'background_g': LaunchConfiguration('background_g'),
            'background_b': LaunchConfiguration('background_b'),
         }]
      ),
   ])
```

## 2.2 Loading parameters from YAML file

We'll start a new simulation of the turtlesim with a different setup during the second launch. Making a turtlesim_world_2.launch.py file is the next step.

```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_tutorial'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         namespace='turtlesim2',
         name='sim',
         parameters=[config]
      )
   ])
```

Let's now make turtlesim.yaml, a configuration file that will be loaded by our launch file, in the /config subdirectory of our package.

```
/turtlesim2/sim:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
```

## 2.3 Using wildcards in YAML files

Let's now make a new file called turtlesim_world_3.launch.py that is similar to turtlesim_world_2.launch.py and adds a third turtlesim node node.

```
...
Node(
   package='turtlesim',
   executable='turtlesim_node',
   namespace='turtlesim3',
   name='sim',
   parameters=[config]
)
```

We will now make the following changes to the turtlesim.yaml file in the /config folder:

```
/**:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
```

## 3. Namespaces

We must first remove the namespace='turtlesim2' line from the turtlesim_world_2.launch.py file because every nested node would immediately inherit that namespace. The launch_turtlesim.launch.py needs to be updated to include the following lines:

```
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

   ...
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
```
Finally, we change the return LaunchDescription statement's turtlesim_world_2 to turtlesim_world_2_with_namespace. As a result, each node in the turtlesim_world_2.launch.py launch description will have a turtlesim2 namespace.

## 4. Reusing nodes

A broadcaster_listener.launch.py file should now be created.

```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster1',
         parameters=[
            {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster2',
         parameters=[
            {'turtlename': 'turtle2'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_listener',
         name='listener',
         parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])
```

## 5. Remapping

A mimic.launch.py file should now be created.

```
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='mimic',
         name='mimic',
         remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
         ]
      )
   ])
```

## 6. Config files

Now let's make a turtlesim_rviz.launch.py file.

```
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   rviz_config = os.path.join(
      get_package_share_directory('turtle_tf2_py'),
      'rviz',
      'turtle_rviz.rviz'
      )

   return LaunchDescription([
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', rviz_config]
      )
   ])
```

## 7. Environment Variables

Now let's construct fixed_broadcaster.launch.py, the final launch file in our package.

```
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='prefix for node name'
      ),
      Node(
            package='turtle_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
      ),
   ])
```

# Running launch files

## 1. Update setup.py

The following lines should be added to setup.py in order to install the launch files from the launch/ folder and the configuration file from the config/ folder. Now, the data files field ought to resemble this:

```
data_files=[
      ...
      (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
      (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
   ],
```

## 2. Build and run

Build the package and start the top-level launch file using the following command to view the outcome of our code at last:

```
ros2 launch launch_tutorial launch_turtlesim.launch.py
```

![image](https://user-images.githubusercontent.com/92040822/197096815-775d19ad-e263-49ae-9584-21a383707d55.png)


Use the teleop node to command the turtle1.

```
ros2 run turtlesim turtle_teleop_key
```

![image](https://user-images.githubusercontent.com/92040822/197095974-bbf82568-6e32-4d3a-a4c1-0b78056ef3d3.png)

# Introducing tf2

The objective is to run a turtlesim demo and use a multi-robot example to demonstrate some of the potential of tf2.

## Installing the demo

Install the demonstration packages and any prerequisites.

```
sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
```

![image](https://user-images.githubusercontent.com/92040822/197098236-a4aee543-1864-4b00-955e-20eee30d9478.png)

## Running the demo

Open a new terminal and source your ROS 2 installation after installing the turtle_tf2_py instruction package. then issue the following command:

```
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```

The turtle simulation will begin with two turtles.

![image](https://user-images.githubusercontent.com/92040822/197098789-ee6878bf-dc2e-426d-a70a-e2aac8ef7c27.png)

You should enter the following command into the second terminal window:

```
ros2 run turtlesim turtle_teleop_key
```

You may observe that one turtle follows the turtle you are driving around repeatedly.

![image](https://user-images.githubusercontent.com/92040822/197099203-2e9b655d-fca0-4405-85a6-92d0aaabfc70.png)


## What is taking place?

The three coordinate frames in this demonstration: a world frame, a turtle1 frame, and a turtle2 frame were made using the tf2 library. In this lesson, the turtle coordinate frames are published by a tf2 broadcaster, and the turtle coordinate frames are then compared by a tf2 listener, which then causes one turtle to follow the other.

## tf2 tools

We can examine what tf2 is doing in the background using tf2 tools.

## 1. Using view_frames

view_frames generates a graphic of the frames that tf2 is transmitting over ROS.

```
ros2 run tf2_tools view_frames.py
```

You will see:

```
Listening to tf data during 5 seconds...
Generating graph in frames.pdf file...
```

![image](https://user-images.githubusercontent.com/92040822/197099940-12999863-fbfb-471f-a9ad-1d8fa4b6719e.png)

## 2. Using tf2_echo

The transform between any two frames broadcast over ROS is reported by the function tf2_echo.

Usage:

```
ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]
```

![image](https://user-images.githubusercontent.com/92040822/197100357-86c5d287-2474-42bf-8839-f3f073c11f17.png)

Let’s look at the transform of the turtle2 frame with respect to turtle1 frame which is equivalent to:

```
ros2 run tf2_ros tf2_echo turtle2 turtle1
```

The transform will be visible as soon as the tf2 echo listener receives the frames sent via ROS2.

```
At time 1622031731.625364060
- Translation: [2.796, 1.039, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.202, 0.979]
At time 1622031732.614745114
- Translation: [1.608, 0.250, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.032, 0.999]
```

![image](https://user-images.githubusercontent.com/92040822/197100423-ac259658-f63f-41ef-848f-248bc91d1492.png)

You can vary the transform as you maneuver your turtle around by moving the two turtles in relation to one another.

## rviz and tf2

Rviz is a visualization tool that is helpful for looking at tf2 frames. Let's use rviz to inspect our turtle frames. Let's start rviz using the -d option and the turtle_rviz.rviz configuration file:

```
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
```

![image](https://user-images.githubusercontent.com/92040822/197100915-7b091667-c5c2-48cf-b1f7-714497bb9299.png)

You can view the frames that tf2 broadcasts in the sidebar. The frames will move in rviz as you maneuver the turtle about.


