---
sidebar_position: 11
---

# Chapter 10: Robot Navigation and Path Planning

## Introduction

Robots need to move from point A to point B while avoiding obstacles. This chapter covers navigation, path planning, and how humanoid robots navigate differently from wheeled robots.

## What is Robot Navigation?

**Navigation** = Getting from here to there safely

**Three main parts:**
1. **Localization**: Where am I?
2. **Mapping**: What does the world look like?
3. **Path Planning**: How do I get there?

**Think of it like GPS:**
- GPS knows where you are (localization)
- Has a map (mapping)
- Plans your route (path planning)

## Localization: Where Am I?

### Why Localization Matters

**Without knowing position:**
- Can't plan paths
- Don't know where obstacles are relative to you
- Can't reach goals

**With good localization:**
- Know exact position
- Can plan accurately
- Navigate efficiently

### Methods of Localization

**1. Odometry**
- Tracks wheel/motor movement
- Estimates position from movement
- Simple but accumulates errors

**2. Visual SLAM**
- Uses cameras
- Creates map and tracks position
- More accurate

**3. LiDAR SLAM**
- Uses LiDAR
- Very accurate
- Works in dark

**4. GPS (for outdoor)**
- Satellite positioning
- Very accurate outdoors
- Doesn't work indoors

**5. Beacons/Markers**
- Fixed markers in environment
- Robot detects them
- Knows position relative to markers

### Combining Methods

**Best practice**: Use multiple methods
- Odometry: Fast updates
- Visual/LiDAR SLAM: Accurate
- GPS: Global reference
- Fusion improves reliability

## Mapping: What Does the World Look Like?

### Types of Maps

**1. Occupancy Grid Map**
- Divides space into grid cells
- Each cell: free or occupied
- Simple and effective

**2. Point Cloud Map**
- 3D points from LiDAR
- Detailed 3D structure
- More complex

**3. Topological Map**
- Graph of places and connections
- Like a subway map
- High-level navigation

**4. Semantic Map**
- Includes object labels
- Knows what things are
- More intelligent

### Building Maps

**SLAM Process:**
1. Robot moves around
2. Sensors collect data
3. Algorithm builds map
4. Simultaneously tracks position
5. Map improves over time

**Pre-built Maps:**
- Sometimes maps exist
- Robot just localizes
- Faster and easier

## Path Planning: How Do I Get There?

### What Path Planning Does

**Given:**
- Start position
- Goal position
- Map of environment
- Obstacles

**Find:**
- Safe path
- Efficient route
- Executable by robot

### Types of Path Planning

**1. Global Planning**
- Plans entire path
- Uses complete map
- Optimal route

**2. Local Planning**
- Plans short-term
- Reacts to obstacles
- Handles surprises

**3. Hybrid Approach**
- Global plan + local adjustments
- Best of both worlds
- Most common

## Nav2: ROS 2 Navigation Stack

### What is Nav2?

**Nav2** is the standard ROS 2 navigation system:
- Complete navigation solution
- Works with many robots
- Well tested and documented
- Industry standard

### Nav2 Components

**1. Map Server**
- Loads and serves maps
- Provides map data

**2. AMCL (Localization)**
- Adaptive Monte Carlo Localization
- Tracks robot position
- Works with laser/vision

**3. Planner**
- Plans global path
- A* or other algorithms
- Finds optimal route

**4. Controller**
- Follows the path
- Avoids obstacles
- Smooth movement

**5. Recovery Behaviors**
- What to do if stuck
- Backup, rotate, etc.
- Gets unstuck

### Using Nav2

**Basic Setup:**
1. Provide map
2. Configure robot parameters
3. Set up sensors
4. Launch Nav2
5. Send goals

**Sending Goals:**
```python
from nav2_msgs.action import NavigateToPose

# Create goal
goal = NavigateToPose.Goal()
goal.pose.pose.position.x = 5.0
goal.pose.pose.position.y = 3.0

# Send to Nav2
action_client.send_goal(goal)
```

## Path Planning Algorithms

### A* Algorithm

**What it does:**
- Finds optimal path
- Considers distance and obstacles
- Very popular

**How it works:**
1. Explores from start
2. Evaluates paths
3. Chooses best option
4. Continues to goal

**Pros:**
- Optimal solution
- Efficient
- Well understood

**Cons:**
- Needs complete map
- Can be slow for large maps

### RRT (Rapidly-exploring Random Tree)

**What it does:**
- Explores space randomly
- Builds tree of paths
- Good for complex spaces

**Pros:**
- Works in complex environments
- Handles high dimensions
- Fast

**Cons:**
- Not always optimal
- Can be jerky

### DWA (Dynamic Window Approach)

**What it does:**
- Plans locally
- Considers robot dynamics
- Real-time reactive

**Pros:**
- Fast
- Handles dynamics
- Good for local planning

**Cons:**
- Local only
- Can get stuck

## Humanoid Robot Navigation

### Special Challenges

**1. Balance**
- Must stay balanced
- Can't stop instantly
- Momentum matters

**2. Foot Placement**
- Must step carefully
- Avoid obstacles with feet
- Plan foot steps

**3. Bipedal Movement**
- Two legs, not wheels
- Different dynamics
- More complex

**4. Height**
- Taller than wheeled robots
- Different obstacle clearance
- Can see over things

### Adapting Navigation

**Footstep Planning:**
- Plans where to step
- Considers balance
- Avoids obstacles

**Gait Planning:**
- Plans walking pattern
- Maintains stability
- Adapts to terrain

**Upper Body:**
- Uses arms for balance
- Can manipulate while moving
- More capabilities

## Obstacle Avoidance

### Types of Obstacles

**Static Obstacles:**
- Walls, furniture
- Don't move
- In the map

**Dynamic Obstacles:**
- People, other robots
- Move around
- Need real-time detection

### Avoidance Strategies

**1. Reactive Avoidance**
- Detect obstacle
- Turn away immediately
- Simple but effective

**2. Predictive Avoidance**
- Predict obstacle movement
- Plan around it
- More sophisticated

**3. Social Navigation**
- Understand human behavior
- Navigate politely
- Important for HRI

## Real-World Considerations

### Sensor Limitations

**Cameras:**
- Affected by lighting
- Limited range
- Can miss things

**LiDAR:**
- Expensive
- Can miss glass
- Limited vertical field

**Solution**: Use multiple sensors

### Dynamic Environments

**Challenges:**
- People moving
- Objects changing
- Map becomes outdated

**Solutions:**
- Update map regularly
- Detect changes
- Adapt plans

### Safety

**Critical considerations:**
- Must be safe around people
- Stop if uncertain
- Have emergency stops
- Test thoroughly

## Best Practices

### 1. Good Maps
- Accurate maps
- Update regularly
- Include all obstacles

### 2. Reliable Localization
- Use multiple methods
- Validate position
- Handle failures

### 3. Appropriate Planning
- Choose right algorithm
- Balance global/local
- Consider robot capabilities

### 4. Test Thoroughly
- Test in simulation first
- Test in real environment
- Handle edge cases

## Common Challenges

### Challenge: Getting Lost
**Solutions**: Better localization, multiple methods, recovery behaviors

### Challenge: Stuck Behind Obstacles
**Solutions**: Better planning, recovery behaviors, human help

### Challenge: Dynamic Obstacles
**Solutions**: Real-time detection, predictive planning, social navigation

### Challenge: Narrow Spaces
**Solutions**: Precise control, careful planning, sometimes can't fit

## Summary

- **Navigation**: Localization + Mapping + Path Planning
- **Nav2**: Standard ROS 2 navigation stack
- **Algorithms**: A*, RRT, DWA for different needs
- **Humanoids**: Special considerations for balance and foot placement
- **Obstacles**: Static and dynamic, need different strategies
- **Best Practice**: Good maps, reliable localization, appropriate planning

## What's Next?

Now let's learn about Vision-Language-Action systems—combining AI language models with robots! Move to [Part 5: Vision-Language-Action (VLA)](../part5/chapter11)!





