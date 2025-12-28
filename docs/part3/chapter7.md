---
sidebar_position: 8
---

# Chapter 7: High-Quality Visualization with Unity

## Introduction

While Gazebo is great for physics simulation, Unity excels at creating beautiful, realistic visuals. This chapter shows you how to use Unity for robot visualization and human-robot interaction.

## What is Unity?

**Unity** is a game engine—software used to create video games. But it's also perfect for:
- Robot visualization
- Training simulations
- Human-robot interaction studies
- Creating realistic environments

### Why Use Unity?

**Advantages:**
- **Beautiful Graphics**: Photorealistic rendering
- **Great Tools**: Easy to build environments
- **Performance**: Optimized for real-time graphics
- **Community**: Lots of resources and assets
- **Cross-Platform**: Works on many systems

**When to Use Unity:**
- Need realistic visuals
- Human-robot interaction studies
- Training computer vision models
- Creating demos and presentations

## Unity vs. Gazebo

### Gazebo Strengths
- Better physics simulation
- Built for robotics
- ROS 2 integration
- Free and open-source

### Unity Strengths
- Better graphics
- Easier environment building
- Great for visualization
- Large asset library

### Best Practice: Use Both!
- **Gazebo**: For physics and control
- **Unity**: For visualization and HRI studies

## Setting Up Unity for Robotics

### Installation

1. Download Unity Hub
2. Install Unity Editor (2021.3 LTS or newer)
3. Install ROS 2 integration packages

### ROS 2 Integration

Use packages like:
- **ROS-TCP-Connector**: Connects Unity to ROS 2
- **Unity Robotics Hub**: Official Unity robotics tools

## Basic Unity Concepts

### Scene

A **Scene** is like a level in a game:
- Contains all objects
- Defines the environment
- Can have multiple scenes

### GameObject

A **GameObject** is any object in Unity:
- Robots
- Environment objects
- Cameras
- Lights

### Component

A **Component** adds functionality:
- Transform (position, rotation)
- Mesh Renderer (makes it visible)
- Rigidbody (physics)
- Scripts (custom behavior)

### Prefab

A **Prefab** is a reusable template:
- Create robot once
- Use it many times
- Changes apply to all instances

## Creating a Robot in Unity

### Step 1: Import Robot Model

1. Get 3D model (FBX, OBJ, etc.)
2. Import into Unity
3. Set up materials and textures

### Step 2: Add Physics

1. Add Rigidbody component
2. Add Colliders
3. Configure physics properties

### Step 3: Add ROS 2 Connection

1. Install ROS-TCP-Connector
2. Add ROS connection script
3. Subscribe to topics (like /cmd_vel)
4. Publish sensor data

### Step 4: Add Controllers

1. Create scripts for movement
2. Connect to ROS 2 topics
3. Control robot joints/motors

## Example: Simple Robot Controller

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using geometry_msgs.msg;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Twist>(topicName, MoveRobot);
    }
    
    void MoveRobot(Twist message)
    {
        // Convert ROS message to Unity movement
        float linearX = (float)message.linear.x;
        float angularZ = (float)message.angular.z;
        
        // Move robot
        transform.Translate(0, 0, linearX * Time.deltaTime);
        transform.Rotate(0, angularZ * Mathf.Rad2Deg * Time.deltaTime, 0);
    }
}
```

## Creating Realistic Environments

### Using Unity Assets

Unity Asset Store has:
- Environments (rooms, buildings, outdoor)
- Materials and textures
- 3D models
- Many free assets!

### Building Custom Environments

1. **Terrain Tool**: Create landscapes
2. **ProBuilder**: Build structures
3. **Materials**: Make things look real
4. **Lighting**: Set up realistic lights

### Example: Indoor Environment

```
Room
├── Floor (with texture)
├── Walls (4 walls)
├── Ceiling
├── Furniture (tables, chairs)
├── Lighting (windows, lamps)
└── Objects (for robot to interact with)
```

## Simulating Sensors in Unity

### Camera Simulation

Unity cameras can:
- Render realistic images
- Simulate different lenses
- Add noise and distortion
- Export to ROS 2 topics

**Example:**
```csharp
public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    Camera cam;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        cam = GetComponent<Camera>();
    }
    
    void Update()
    {
        // Capture image
        RenderTexture rt = new RenderTexture(640, 480, 24);
        cam.targetTexture = rt;
        Texture2D image = new Texture2D(640, 480);
        
        // Convert to ROS message and publish
        // (implementation details here)
    }
}
```

### LiDAR Simulation

Unity can simulate LiDAR by:
- Casting rays from sensor
- Measuring distances
- Creating point clouds
- Publishing to ROS 2

## Human-Robot Interaction in Unity

### Why Unity for HRI?

- **Realistic Humans**: Can add human avatars
- **Natural Interaction**: Gestures, expressions
- **User Studies**: Easy to test with people
- **Visualization**: See interactions clearly

### Adding Human Avatars

1. Import human model
2. Add animation (walking, gestures)
3. Add AI behavior
4. Connect to robot interaction

### Example HRI Scenario

**Scenario**: Robot helping human
1. Human approaches robot
2. Robot detects human
3. Robot greets human
4. Human gives command
5. Robot responds and acts

All visualized beautifully in Unity!

## ROS 2 Integration

### Setting Up Connection

1. Install ROS-TCP-Connector package
2. Configure IP address and port
3. Start ROS 2 bridge
4. Connect Unity to ROS 2

### Publishing Topics

```csharp
ros.Publish(topicName, message);
```

### Subscribing to Topics

```csharp
ros.Subscribe<TopicType>(topicName, CallbackFunction);
```

## Performance Optimization

### Tips for Better Performance

1. **Use LODs**: Different detail levels
2. **Occlusion Culling**: Don't render hidden objects
3. **Batching**: Combine similar objects
4. **Reduce Polygons**: Simplify models
5. **Optimize Textures**: Compress images

### Target Frame Rate

- **Real-time**: 30-60 FPS
- **Training**: Can be slower
- **Rendering**: Can be offline

## Common Workflows

### Workflow 1: Visualization Only

1. Import robot model
2. Create environment
3. Connect to ROS 2
4. Visualize robot movement
5. No physics simulation

### Workflow 2: Training Data Generation

1. Set up scene
2. Add variations (lighting, objects)
3. Capture images/videos
4. Export for AI training
5. Generate synthetic datasets

### Workflow 3: HRI Studies

1. Create realistic environment
2. Add human avatars
3. Set up interaction scenarios
4. Record interactions
5. Analyze results

## Best Practices

### 1. Organize Your Project

```
Assets/
├── Robots/
│   └── MyRobot/
├── Environments/
│   └── Room1/
├── Scripts/
│   └── Controllers/
└── Materials/
    └── Textures/
```

### 2. Use Version Control

- Git LFS for large files
- Keep scenes organized
- Document your setup

### 3. Test Regularly

- Test ROS 2 connection
- Verify robot movement
- Check sensor outputs

### 4. Optimize Early

- Don't wait until end
- Profile performance
- Fix bottlenecks early

## Common Challenges

### Challenge: ROS 2 Connection Issues
**Solution**: Check IP, port, firewall settings

### Challenge: Performance Problems
**Solution**: Optimize graphics, reduce complexity

### Challenge: Physics Not Accurate
**Solution**: Use Gazebo for physics, Unity for visuals

### Challenge: Complex Setup
**Solution**: Start with examples, build gradually

## Summary

- **Unity**: Great for visualization and graphics
- **Use Cases**: HRI, training data, demos
- **ROS 2 Integration**: Connect via ROS-TCP-Connector
- **Best Practice**: Use Unity for visuals, Gazebo for physics
- **Performance**: Optimize for your needs

## What's Next?

Now let's learn about NVIDIA Isaac—the most advanced platform for robot AI! Move to [Part 4: The AI Robot Brain (NVIDIA Isaac)](../part4/chapter8)!





