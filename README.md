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

## 7. 8 Environment Variables

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

Use the teleop node to command the turtle1.

```
ros2 run turtlesim turtle_teleop_key
```

