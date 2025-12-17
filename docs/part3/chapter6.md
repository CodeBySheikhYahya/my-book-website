---
sidebar_position: 7
---

# Chapter 6: Building Virtual Worlds with Gazebo

## Introduction

Now that you understand why simulation matters, let's learn how to actually build virtual worlds and robots in Gazebo. This chapter will teach you the practical skills you need.

## Understanding URDF and SDF

### URDF: Robot Description Format

**URDF** (Unified Robot Description Format) describes your robot using XML.

**What it describes:**
- **Links**: The rigid parts (like bones)
- **Joints**: How parts connect and move
- **Visual**: How it looks
- **Collision**: Shape for physics
- **Inertial**: Mass and balance

### SDF: Simulation Description Format

**SDF** describes entire simulation worlds, including:
- Robots
- Environments
- Objects
- Physics settings
- Lighting

**Think of it as:**
- URDF = Describes one robot
- SDF = Describes entire simulation scene

## Creating Your First Robot Model

### Simple Example: A Box Robot

Let's create a simple robot that's just a box with wheels:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link (the main body) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base to left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (similar) -->
  <link name="right_wheel">
    <!-- Similar to left wheel -->
  </link>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

**What this creates:**
- A box-shaped body
- Two wheels that can rotate
- Basic physics properties

## Understanding Physics Simulation

### Gravity

Gravity makes things fall. In Gazebo:
```xml
<gravity>0 0 -9.8</gravity>
```
This means: 9.8 m/s² downward (like Earth)

### Collisions

When objects touch:
- Gazebo calculates forces
- Objects push each other
- Can bounce or slide

### Friction

Friction affects movement:
- **High friction**: Hard to slide
- **Low friction**: Easy to slide
- Set in material properties

### Mass and Inertia

**Mass**: How heavy something is
**Inertia**: How hard it is to rotate

Both affect how objects move!

## Creating Environments

### Building a Simple Room

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_room">
    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="wall1">
      <pose>2 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision>
          <geometry>
            <box>
              <size>0.1 4 2</size>
            </box>
          </geometry>
        </collision>
        <visual>
          <geometry>
            <box>
              <size>0.1 4 2</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Add more walls, objects, etc. -->
  </world>
</sdf>
```

## Simulating Sensors

### Adding a Camera

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### Adding LiDAR

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Launch Files: Starting Everything

### Basic Launch File

```xml
<?xml version="1.0"?>
<launch>
  <!-- Start Gazebo with empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find my_package)/worlds/my_world.world"/>
  </include>

  <!-- Spawn robot -->
  <param name="robot_description" command="$(find xacro)/xacro $(find my_package)/urdf/my_robot.urdf.xacro"/>
  
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
        args="-param robot_description -urdf -model my_robot" />
</launch>
```

**What this does:**
1. Starts Gazebo
2. Loads your world
3. Spawns your robot
4. Everything ready to go!

## Practical Example: Mobile Robot

### Step 1: Create Robot URDF

Define:
- Base (body)
- Two drive wheels
- Caster wheel (for balance)
- Camera
- LiDAR

### Step 2: Create World File

Define:
- Ground plane
- Walls and obstacles
- Lighting
- Physics settings

### Step 3: Create Launch File

Starts:
- Gazebo
- World
- Robot
- ROS 2 nodes

### Step 4: Test

Run:
```bash
ros2 launch my_package simulation.launch.py
```

Control robot:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist ...
```

## Tips for Building Good Simulations

### 1. Start Simple
- Begin with basic shapes
- Add complexity gradually
- Test as you build

### 2. Use Realistic Values
- Realistic masses
- Proper dimensions
- Accurate physics

### 3. Organize Your Files
```
my_package/
├── urdf/
│   └── robot.urdf
├── worlds/
│   └── world.sdf
└── launch/
    └── simulation.launch.py
```

### 4. Test Incrementally
- Test each part separately
- Verify physics works
- Check sensors output data

### 5. Use Pre-made Models
- Gazebo model database
- ROS packages with models
- Don't reinvent the wheel

## Common Problems and Solutions

### Problem: Robot Falls Through Floor
**Solution**: Check collision geometry, ensure ground exists

### Problem: Robot Too Heavy/Light
**Solution**: Adjust mass values in inertial properties

### Problem: Joints Not Moving
**Solution**: Check joint types, add controllers/plugins

### Problem: Sensors Not Working
**Solution**: Verify plugins are loaded, check topic names

### Problem: Simulation Too Slow
**Solution**: Reduce graphics quality, simplify models

## Advanced Topics

### Using Xacro

**Xacro** makes URDF files easier:
- Variables and parameters
- Macros (reusable parts)
- Math expressions
- Includes other files

**Example:**
```xml
<xacro:macro name="wheel" params="prefix">
  <link name="${prefix}_wheel">
    <!-- wheel definition -->
  </link>
</xacro:macro>

<xacro:wheel prefix="left"/>
<xacro:wheel prefix="right"/>
```

### Mesh Files

Instead of simple shapes, use 3D models:
```xml
<visual>
  <geometry>
    <mesh filename="package://my_package/meshes/robot_body.dae"/>
  </geometry>
</visual>
```

## Summary

- **URDF**: Describes robot structure
- **SDF**: Describes entire simulation
- **Physics**: Gravity, collisions, friction
- **Sensors**: Cameras, LiDAR, IMU
- **Launch Files**: Start everything together
- **Best Practice**: Start simple, build gradually

## What's Next?

Now let's learn about Unity for high-quality visualization! Move to [Chapter 7: High-Quality Visualization with Unity](./chapter7)!



