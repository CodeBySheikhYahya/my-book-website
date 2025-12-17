---
sidebar_position: 5
---

# Chapter 4: Connecting Python AI to Robots

## Introduction

You know Python, and you know ROS 2. Now let's connect them! In this chapter, we'll learn how to use Python to control robots through ROS 2.

## Why Python for Robotics?

### Python is Popular in AI

Most AI tools use Python:
- Machine learning libraries (TensorFlow, PyTorch)
- Computer vision (OpenCV)
- Natural language processing (Transformers)
- Data science (NumPy, Pandas)

### Python is Easy to Learn

- Simple, readable syntax
- Great for beginners
- Lots of tutorials and resources
- Large community

### Python Works with ROS 2

ROS 2 has excellent Python support through **rclpy** (ROS 2 Client Library for Python).

## What is rclpy?

**rclpy** is the Python library that lets you:
- Create ROS 2 nodes in Python
- Publish and subscribe to topics
- Call and provide services
- Control robots from Python code

**Think of it as:** A translator between Python and ROS 2

## Setting Up Your Python Environment

### Step 1: Install ROS 2

Make sure ROS 2 Humble (or Iron) is installed on Ubuntu 22.04.

### Step 2: Source ROS 2

Every time you open a terminal:
```bash
source /opt/ros/humble/setup.bash
```

Or add it to your `.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 3: Install Python Dependencies

```bash
pip3 install rclpy
```

## Your First Python ROS 2 Node

Let's create a simple node that publishes messages:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        # Create a publisher
        self.publisher = self.create_publisher(
            String,           # Message type
            'chatter',        # Topic name
            10               # Queue size
        )
        # Create a timer to publish every second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main():
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What this does:**
- Creates a node called "talker"
- Publishes "Hello ROS 2!" messages every second
- Logs what it's doing

## Creating a Listener Node

Now let's create a node that listens:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        # Create a subscriber
        self.subscription = self.create_subscription(
            String,           # Message type
            'chatter',       # Topic name
            self.listener_callback,
            10               # Queue size
        )
        
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What this does:**
- Listens to the "chatter" topic
- Prints whatever it hears

## Running Your Nodes

### Terminal 1: Run the Talker
```bash
python3 talker.py
```

### Terminal 2: Run the Listener
```bash
python3 listener.py
```

You should see messages flowing from talker to listener!

## Controlling a Robot: Example

Let's create a node that controls a robot's movement:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Publish movement commands
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Standard topic for robot velocity
            10
        )
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.control_loop)
        
    def control_loop(self):
        msg = Twist()
        # Move forward at 0.5 m/s
        msg.linear.x = 0.5
        # Turn slightly
        msg.angular.z = 0.1
        self.publisher.publish(msg)
        self.get_logger().info('Moving robot...')

def main():
    rclpy.init()
    controller = RobotController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What this does:**
- Publishes movement commands
- Robot moves forward and turns slightly
- Commands sent 10 times per second

## Reading Sensor Data

Let's read data from a camera:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Now you can process the image with OpenCV
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}')

def main():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Using AI Models with ROS 2

Here's how to use an AI model (like object detection) with ROS 2:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
from cv_bridge import CvBridge
# Assume you have an AI model loaded
# from your_ai_model import detect_objects

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision')
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10
        )
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        self.bridge = CvBridge()
        
    def process_image(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run AI model (your code here)
        # detections = detect_objects(cv_image)
        
        # Publish results
        # detection_msg = self.create_detection_msg(detections)
        # self.detection_pub.publish(detection_msg)
        
        self.get_logger().info('Processed image with AI')

def main():
    rclpy.init()
    node = AIVisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a ROS 2 Package

To organize your code properly, create a ROS 2 package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_package
```

**Package structure:**
```
my_robot_package/
├── package.xml
├── setup.py
└── my_robot_package/
    └── nodes/
        ├── __init__.py
        └── my_node.py
```

## Best Practices

### 1. Use Proper Message Types
- Don't create custom messages unless necessary
- Use standard ROS 2 message types when possible

### 2. Handle Errors
```python
try:
    # Your code
except Exception as e:
    self.get_logger().error(f'Error: {e}')
```

### 3. Use Logging
```python
self.get_logger().info('Information')
self.get_logger().warn('Warning')
self.get_logger().error('Error')
```

### 4. Clean Shutdown
```python
def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Common Patterns

### Pattern 1: Sensor → Processing → Action
```
Camera Node → AI Processing Node → Motor Control Node
```

### Pattern 2: Multiple Sensors → Fusion → Decision
```
Camera + LiDAR → Sensor Fusion Node → Navigation Node
```

### Pattern 3: AI Planning → Execution
```
LLM Planning Node → Action Execution Node → Feedback Node
```

## Debugging Tips

### See What's Happening
```bash
ros2 topic echo /topic_name
ros2 node info /node_name
ros2 topic list
ros2 node list
```

### Check Message Rates
```bash
ros2 topic hz /topic_name
```

### Visualize Data
```bash
rviz2  # ROS 2 visualization tool
```

## Summary

- **rclpy**: Python library for ROS 2
- **Nodes**: Create with `Node` class
- **Publishers**: Send messages on topics
- **Subscribers**: Receive messages from topics
- **Services**: Request-response communication
- **Best Practice**: Organize code in ROS 2 packages

## What's Next?

Now that you can connect Python to ROS 2, let's learn about simulation! Move to [Part 3: Creating Digital Twins (Simulation)](../part3/chapter5) to learn how to test your robots safely in virtual worlds!



