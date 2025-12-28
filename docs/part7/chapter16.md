---
sidebar_position: 17
---

# Chapter 16: Cloud vs. Local Setup

## Introduction

You have two main options for your development environment: build a local lab or use cloud computing. This chapter helps you decide and provides setup instructions for both.

## Comparison: Cloud vs. Local

### Quick Comparison

| Aspect | Local Lab | Cloud Lab |
|--------|-----------|-----------|
| **Upfront Cost** | High ($1,500-$3,000) | Low ($0) |
| **Ongoing Cost** | Low (electricity) | Medium ($200+/quarter) |
| **Performance** | Consistent | Depends on instance |
| **Internet** | Not required | Required |
| **Availability** | Always | When paid |
| **Control** | Full | Limited |
| **Setup Time** | Days | Hours |

## Option 1: Local Lab Setup

### Advantages

**Full Control:**
- Your hardware, your rules
- No internet dependency
- Customize everything
- No usage limits

**Performance:**
- Consistent performance
- No network latency
- Full GPU access
- Fast local storage

**Cost Over Time:**
- One-time purchase
- No monthly fees
- Can resell hardware
- Better long-term value

### Disadvantages

**Upfront Cost:**
- Significant investment
- $1,500-$3,000 minimum
- Can't try before buying

**Maintenance:**
- You maintain hardware
- Troubleshooting yourself
- Software updates
- Potential repairs

**Space:**
- Need physical space
- Power requirements
- Cooling considerations

### When to Choose Local

✅ **Choose local if:**
- Budget allows upfront cost
- Long-term use planned
- Want full control
- Don't want ongoing fees
- Have space available

### Setup Steps

**1. Purchase Hardware**
- GPU: RTX 4070 Ti or better
- CPU: Intel i7/AMD Ryzen 9
- RAM: 32GB minimum (64GB recommended)
- SSD: 1TB+ NVMe
- Motherboard: Compatible with GPU
- PSU: 850W+ (for high-end GPU)

**2. Install Ubuntu 22.04**
- Download ISO
- Create bootable USB
- Install on system
- Update system

**3. Install NVIDIA Drivers**
```bash
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot
```

**4. Install CUDA**
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run
sudo sh cuda_12.2.0_535.54.03_linux.run
```

**5. Install ROS 2 Humble**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-humble-desktop
```

**6. Install Isaac Sim**
- Download NVIDIA Omniverse Launcher
- Install Isaac Sim through launcher
- Verify installation

**7. Install Development Tools**
```bash
sudo apt install git python3-pip build-essential
pip3 install openai-whisper openai rclpy
```

## Option 2: Cloud Lab Setup

### Advantages

**Low Upfront Cost:**
- No hardware purchase
- Pay as you go
- Try before committing
- Access powerful GPUs

**Easy Setup:**
- Pre-configured instances
- Quick to start
- No hardware issues
- Scalable

**Flexibility:**
- Use when needed
- Stop when done
- Try different configurations
- No maintenance

### Disadvantages

**Ongoing Costs:**
- Monthly/quarterly fees
- Can add up over time
- More expensive long-term

**Dependencies:**
- Requires internet
- Network latency
- Limited control
- Instance availability

**Real Robot Issues:**
- Can't control real robot from cloud
- Latency too high
- Need local edge kit anyway

### When to Choose Cloud

✅ **Choose cloud if:**
- Limited upfront budget
- Short-term project
- Want to try first
- Don't have space
- Need flexibility

### Setup Steps: AWS

**1. Create AWS Account**
- Sign up at aws.amazon.com
- Set up billing alerts
- Choose region (us-east-1 recommended)

**2. Launch GPU Instance**
- EC2 → Launch Instance
- Choose: g5.2xlarge or g6e.xlarge
- Select Ubuntu 22.04 AMI
- Configure storage (100GB+)
- Launch instance

**3. Connect to Instance**
```bash
ssh -i your-key.pem ubuntu@your-instance-ip
```

**4. Install NVIDIA Drivers**
```bash
sudo apt update
sudo apt install -y nvidia-driver-535
sudo reboot
# Wait for reboot, reconnect
```

**5. Install CUDA**
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run
sudo sh cuda_12.2.0_535.54.03_linux.run
```

**6. Install ROS 2 Humble**
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-humble-desktop
```

**7. Install Isaac Sim**
- Download Omniverse Launcher
- Install Isaac Sim
- Note: May need specific AMI

**8. Set Up Remote Access**
- Use VNC or X11 forwarding
- Or use VS Code Remote SSH
- Configure for GUI applications

### Cost Management

**Tips to Save Money:**
- Use spot instances (cheaper)
- Stop instances when not using
- Use smaller instances when possible
- Monitor usage closely
- Set up billing alerts

**Estimated Costs:**
- g5.2xlarge: ~$1.50/hour
- 10 hours/week: ~$60/month
- 12 weeks: ~$180/quarter
- Storage: ~$25/quarter
- **Total**: ~$205/quarter

## Hybrid Approach

### Best of Both Worlds

**Use cloud for:**
- Heavy simulation workloads
- AI model training
- Testing different configurations

**Use local for:**
- Development and coding
- Quick testing
- Real robot control
- When internet is down

### Setup

**Local Machine:**
- Laptop or desktop
- For coding and light work
- Connect to cloud for heavy tasks

**Cloud Instance:**
- For simulations
- For training
- Access remotely

**Edge Kit:**
- For real robot
- Local processing
- Low latency control

## Making Your Decision

### Decision Matrix

**Choose Local If:**
- ✅ Budget > $1,500
- ✅ Using > 6 months
- ✅ Want full control
- ✅ Have space
- ✅ Don't want ongoing costs

**Choose Cloud If:**
- ✅ Budget < $1,500
- ✅ Using < 6 months
- ✅ Want flexibility
- ✅ Limited space
- ✅ Okay with ongoing costs

**Choose Hybrid If:**
- ✅ Want both benefits
- ✅ Have some budget
- ✅ Need flexibility
- ✅ Want to optimize costs

## Setup Checklist

### Local Setup Checklist

- [ ] Purchase hardware components
- [ ] Assemble computer
- [ ] Install Ubuntu 22.04
- [ ] Install NVIDIA drivers
- [ ] Install CUDA
- [ ] Install ROS 2 Humble
- [ ] Install Isaac Sim
- [ ] Install development tools
- [ ] Test basic simulation
- [ ] Verify GPU acceleration

### Cloud Setup Checklist

- [ ] Create AWS account
- [ ] Set up billing alerts
- [ ] Launch GPU instance
- [ ] Connect via SSH
- [ ] Install NVIDIA drivers
- [ ] Install CUDA
- [ ] Install ROS 2 Humble
- [ ] Install Isaac Sim
- [ ] Configure remote access
- [ ] Test basic simulation
- [ ] Set up cost monitoring

## Troubleshooting

### Local Issues

**GPU not detected:**
- Check driver installation
- Verify GPU is seated properly
- Check power connections

**Simulation too slow:**
- Check GPU usage
- Reduce scene complexity
- Close other applications

**ROS 2 not working:**
- Verify installation
- Check environment setup
- Source ROS 2 setup

### Cloud Issues

**Can't connect:**
- Check security groups
- Verify key permissions
- Check instance status

**Performance issues:**
- Check instance type
- Monitor GPU usage
- Consider larger instance

**Cost too high:**
- Use spot instances
- Stop when not using
- Monitor usage

## Summary

- **Local Lab**: High upfront, low ongoing, full control
- **Cloud Lab**: Low upfront, ongoing costs, flexible
- **Hybrid**: Best of both worlds
- **Choose**: Based on budget, timeline, needs
- **Setup**: Follow checklists for your choice

## What's Next?

Now let's learn about the learning schedule and how to structure your studies! Move to [Part 8: Learning Path and Schedule](../part8/chapter17)!





