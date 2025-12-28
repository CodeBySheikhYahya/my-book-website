---
sidebar_position: 16
---

# Chapter 15: What Hardware Do You Need?

## Introduction

Building Physical AI robots requires specific hardware. This chapter explains what you need, why you need it, and how to choose the right components for your budget and goals.

## Hardware Overview

### Two Main Categories

**1. Development Workstation**
- Powerful computer for simulation
- Runs Isaac Sim, Gazebo, training
- Most important investment

**2. Physical Robot Hardware**
- Edge computing (Jetson)
- Sensors (cameras, LiDAR)
- Actual robot (optional)

## The Digital Twin Workstation (Required)

### Why You Need This

**This is your most important hardware!**

- Runs all simulations
- Trains AI models
- Develops and tests code
- Can't skip this!

### GPU: The Critical Component

**Why GPU matters:**
- Isaac Sim needs RTX graphics
- AI training requires GPU
- Real-time simulation needs power
- This is your bottleneck!

**Minimum Requirements:**
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM)
- **Why**: Can run Isaac Sim and basic AI

**Recommended:**
- **GPU**: NVIDIA RTX 3090 or 4090 (24GB VRAM)
- **Why**: Smooth performance, can train larger models

**Why not cheaper GPUs?**
- Non-RTX cards can't run Isaac Sim
- Not enough VRAM for simulations
- Too slow for real-time work

### CPU Requirements

**What you need:**
- Intel Core i7 (13th Gen+) or AMD Ryzen 9
- **Why**: Physics simulation is CPU-intensive
- Gazebo/Isaac do lots of physics calculations

**Why powerful CPU?**
- Physics calculations need fast CPU
- Multiple cores help
- Can't rely only on GPU

### RAM Requirements

**Minimum**: 32GB DDR5
- **Why**: Complex scenes need memory
- Will crash with less

**Recommended**: 64GB DDR5
- **Why**: Smooth operation
- Can handle large simulations
- Room for growth

**Why so much RAM?**
- USD scene assets are large
- AI models need memory
- Multiple applications running
- Simulation buffers

### Storage

**What you need:**
- **SSD**: 1TB+ NVMe SSD
- **Why**: Fast loading of scenes and models
- Large datasets and simulations

**Additional storage:**
- External drive for backups
- Cloud storage for projects

### Operating System

**Required**: Ubuntu 22.04 LTS
- **Why**: ROS 2 works best on Linux
- Isaac Sim supports Linux best
- Industry standard

**Alternatives:**
- Windows (limited ROS 2 support)
- Mac (can't run Isaac Sim)
- Linux is strongly recommended

### Complete Workstation Specs

**Budget Option:**
- GPU: RTX 4070 Ti (12GB) - $800
- CPU: Intel i7-13700K - $400
- RAM: 32GB DDR5 - $150
- SSD: 1TB NVMe - $100
- **Total**: ~$1,450

**Recommended Option:**
- GPU: RTX 4090 (24GB) - $1,600
- CPU: Intel i9-13900K - $600
- RAM: 64GB DDR5 - $300
- SSD: 2TB NVMe - $200
- **Total**: ~$2,700

## The Physical AI Edge Kit

### What Is This?

**Edge computing kit** = Small computer that runs robot AI
- Like robot's brain
- Tests code before deploying
- Understands resource constraints

### Why You Need This

**Even if you only simulate:**
- Learn about real robot constraints
- Test code on actual hardware
- Understand edge computing
- Prepare for real robots

### The Brain: NVIDIA Jetson

**Options:**

**1. Jetson Orin Nano (8GB) - $249**
- **Best for**: Learning, basic projects
- **Performance**: Good for most tasks
- **Power**: 40 TOPS AI performance

**2. Jetson Orin NX (16GB) - $699**
- **Best for**: Advanced projects
- **Performance**: Much faster
- **Power**: 100 TOPS AI performance

**Why Jetson?**
- Industry standard for robots
- Optimized for AI
- Runs ROS 2 well
- Good community support

### The Eyes: Depth Camera

**Intel RealSense D435i - $349**
- **What it provides**: RGB + Depth vision
- **Why**: Essential for VSLAM and perception
- **Features**: Built-in IMU

**Why RealSense?**
- Good ROS 2 support
- Reliable depth sensing
- Industry standard
- Well documented

### The Ears: Microphone

**ReSpeaker USB Mic Array v2.0 - $69**
- **What it provides**: Voice input
- **Why**: For voice commands (Whisper)
- **Features**: Far-field pickup

**Why this mic?**
- Good for robot applications
- Picks up voice from distance
- USB connection (easy)

### Complete Edge Kit

**Economy Kit:**
- Jetson Orin Nano: $249
- RealSense D435i: $349
- ReSpeaker Mic: $69
- SD Card + Misc: $30
- **Total**: ~$700

**Recommended Kit:**
- Jetson Orin NX: $699
- RealSense D435i: $349
- ReSpeaker Mic: $69
- SD Card + Misc: $30
- **Total**: ~$1,150

## The Robot (Optional)

### Option A: Budget-Friendly (Recommended)

**Unitree Go2 Edu - $1,800-$3,000**
- **Type**: Quadruped (dog-like)
- **Why**: Teaches same principles
- **Pros**: Affordable, durable, good ROS support
- **Cons**: Not humanoid

**Why this is good:**
- 90% of software transfers to humanoids
- Much cheaper than humanoid
- Great for learning
- Can upgrade later

### Option B: Miniature Humanoid

**Hiwonder TonyPi Pro - ~$600**
- **Type**: Small humanoid
- **Why**: Affordable humanoid option
- **Pros**: Actually humanoid, cheap
- **Cons**: Limited capabilities, Raspberry Pi based

**Considerations:**
- Can't run Isaac ROS efficiently
- Good for kinematics learning
- Use Jetson for AI processing

### Option C: Professional Humanoid

**Unitree G1 - ~$16,000**
- **Type**: Full humanoid
- **Why**: Industry-grade robot
- **Pros**: Can actually walk, manipulate
- **Cons**: Very expensive

**When to consider:**
- Serious research
- Commercial applications
- Have budget
- Need real deployment

## Cloud vs. Local Setup

### Option 1: Local Lab (Buy Hardware)

**Pros:**
- Fast (no internet needed)
- Always available
- No ongoing costs
- Full control

**Cons:**
- High upfront cost ($2,000-$3,000)
- Need space
- Maintenance required

**Best for:**
- Serious learning
- Long-term use
- Multiple projects

### Option 2: Cloud Lab (Rent Computing)

**What you rent:**
- AWS g5.2xlarge instance
- GPU-enabled cloud workstation
- ~$1.50/hour

**Cost calculation:**
- 10 hours/week × 12 weeks = 120 hours
- 120 hours × $1.50 = $180/quarter
- Plus storage: ~$25
- **Total**: ~$205/quarter

**Pros:**
- No upfront hardware cost
- Access powerful GPUs
- Scalable
- No maintenance

**Cons:**
- Ongoing costs
- Needs internet
- Latency for real robots
- Less control

**Best for:**
- Short-term projects
- Trying before buying
- Limited budget

**Note**: You still need edge kit and robot for physical deployment!

## Complete Setup Options

### Option 1: Simulation Only (Minimum)

**What you get:**
- Development workstation
- Can do everything in simulation

**Cost**: ~$1,500-$2,700
**Good for**: Learning, development, capstone project

### Option 2: Simulation + Edge Kit

**What you get:**
- Development workstation
- Edge computing kit
- Can test on real hardware

**Cost**: ~$2,200-$3,850
**Good for**: Serious learning, real hardware experience

### Option 3: Complete Lab

**What you get:**
- Development workstation
- Edge computing kit
- Physical robot

**Cost**: ~$4,000-$20,000
**Good for**: Research, commercial, full deployment

## Making the Choice

### Questions to Ask

1. **Budget?**
   - Under $2,000: Simulation only
   - $2,000-$4,000: Add edge kit
   - $4,000+: Add robot

2. **Goals?**
   - Learning: Simulation is enough
   - Research: Need edge kit
   - Deployment: Need robot

3. **Timeline?**
   - Short-term: Consider cloud
   - Long-term: Buy hardware

4. **Space?**
   - Limited: Cloud or small setup
   - Available: Full lab

## Budget Recommendations

### Student Budget (~$1,500)

**Get:**
- RTX 4070 Ti workstation
- 32GB RAM
- 1TB SSD
- Ubuntu 22.04

**Can do:**
- All simulations
- AI training
- Capstone project
- Learn everything

### Professional Budget (~$3,000)

**Get:**
- RTX 4090 workstation
- 64GB RAM
- Edge kit (Jetson + sensors)
- 2TB SSD

**Can do:**
- Everything above
- Real hardware testing
- Advanced projects
- Production-ready development

### Research Budget (~$10,000+)

**Get:**
- High-end workstation
- Edge kit
- Unitree Go2 or G1 robot
- Additional sensors

**Can do:**
- Full deployment
- Real-world testing
- Research projects
- Commercial applications

## Summary

- **Workstation**: Most important, needs powerful GPU
- **Edge Kit**: For real hardware experience
- **Robot**: Optional, depends on goals
- **Cloud**: Alternative to buying hardware
- **Choose**: Based on budget, goals, timeline

## What's Next?

Now let's learn about setting up your development environment! Move to [Chapter 16: Cloud vs. Local Setup](./chapter16) for detailed setup instructions!





