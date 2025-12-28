---
sidebar_position: 13
---

# Chapter 12: AI Planning for Robots

## Introduction

You can tell a robot "clean the room" and it figures out all the steps itself! This chapter shows you how to use Large Language Models (LLMs) to make robots understand complex commands and plan their actions.

## What is AI Planning?

### The Problem

**Simple commands are easy:**
- "Move forward" → Robot moves forward
- "Turn left" → Robot turns left
- "Stop" → Robot stops

**Complex commands are hard:**
- "Clean the room" → What does this mean?
- "Make me breakfast" → Many steps needed
- "Help me organize my desk" → Complex task

### The Solution: AI Planning

**AI Planning** uses Large Language Models (LLMs) to:
- Understand complex commands
- Break them into steps
- Plan the sequence of actions
- Execute the plan

**Think of it like:**
- You tell a human assistant "clean the room"
- They figure out: pick up trash, vacuum, organize, etc.
- They do it step by step
- Same idea for robots!

## What are Large Language Models (LLMs)?

### Simple Explanation

**LLMs** are AI systems trained on huge amounts of text:
- Understand language very well
- Can reason about tasks
- Generate plans and sequences
- Examples: GPT-4, Claude, Llama

### Why LLMs for Robots?

**Advantages:**
- **Understand Natural Language**: Handle complex commands
- **Reasoning**: Can figure out what to do
- **Planning**: Break tasks into steps
- **Flexibility**: Handle new situations
- **Common Sense**: Understand context

## How LLM Planning Works

### The Process

**Step 1: Receive Command**
- User says: "Clean the room"
- Convert speech to text (Whisper)

**Step 2: Understand Intent**
- LLM analyzes the command
- Understands what "clean" means
- Identifies the target (room)

**Step 3: Generate Plan**
- LLM breaks task into steps:
  1. Find trash
  2. Pick up trash
  3. Throw in trash can
  4. Find more trash
  5. Repeat until done

**Step 4: Convert to Actions**
- Each step becomes robot actions
- "Find trash" → Use vision to detect
- "Pick up" → Move arm and grasp
- "Throw away" → Navigate and deposit

**Step 5: Execute**
- Robot performs actions
- Monitors progress
- Adjusts if needed

## Using GPT Models for Planning

### Setting Up OpenAI API

**Installation:**
```bash
pip install openai
```

**Basic Usage:**
```python
import openai

openai.api_key = "your-api-key"

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "You are a robot planning assistant."},
        {"role": "user", "content": "Clean the room"}
    ]
)

plan = response.choices[0].message.content
print(plan)
```

### Creating a Planning Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        openai.api_key = "your-api-key"
        
        # Subscribe to commands
        self.cmd_sub = self.create_subscription(
            String,
            '/voice/command',
            self.plan_command,
            10
        )
        
        # Publish plan
        self.plan_pub = self.create_publisher(
            String,
            '/robot/plan',
            10
        )
        
    def plan_command(self, msg):
        command = msg.data
        self.get_logger().info(f'Planning for: {command}')
        
        # Create prompt for LLM
        prompt = f"""
        You are a robot assistant. Break down this task into steps: {command}
        
        Return a JSON list of steps, each with:
        - action: what to do
        - target: what object/location
        - description: brief description
        """
        
        # Call LLM
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful robot planning assistant. Always return valid JSON."},
                {"role": "user", "content": prompt}
            ]
        )
        
        plan_text = response.choices[0].message.content
        
        # Parse and publish plan
        try:
            plan = json.loads(plan_text)
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            self.get_logger().info(f'Generated plan: {plan}')
        except:
            self.get_logger().error('Failed to parse plan')

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: "Clean the Room" Command

### What the LLM Might Generate

```json
[
  {
    "step": 1,
    "action": "scan",
    "target": "room",
    "description": "Look around the room to find items to clean"
  },
  {
    "step": 2,
    "action": "navigate",
    "target": "trash_location",
    "description": "Move to where trash is located"
  },
  {
    "step": 3,
    "action": "pick",
    "target": "trash",
    "description": "Pick up the trash item"
  },
  {
    "step": 4,
    "action": "navigate",
    "target": "trash_can",
    "description": "Move to the trash can"
  },
  {
    "step": 5,
    "action": "deposit",
    "target": "trash_can",
    "description": "Put trash in the trash can"
  },
  {
    "step": 6,
    "action": "repeat",
    "target": "until_clean",
    "description": "Repeat until room is clean"
  }
]
```

### Converting Plan to Robot Actions

```python
def execute_plan(plan):
    for step in plan:
        action = step['action']
        target = step['target']
        
        if action == 'scan':
            # Use vision to scan room
            objects = scan_room()
            
        elif action == 'navigate':
            # Use Nav2 to navigate
            navigate_to(target)
            
        elif action == 'pick':
            # Use manipulation to pick
            pick_object(target)
            
        elif action == 'deposit':
            # Deposit object
            deposit_object(target)
            
        # Wait for action to complete
        wait_for_completion()
```

## Advanced Planning Features

### Context Awareness

**LLM can consider:**
- Current robot state
- Environment information
- Previous actions
- User preferences

**Example:**
- Robot knows it's in kitchen
- User says "bring me that"
- LLM understands "that" = object in kitchen
- Plans navigation and manipulation

### Dynamic Replanning

**If something goes wrong:**
- LLM can replan
- Adjust to new situation
- Handle failures gracefully

**Example:**
- Plan: Pick up cup
- Problem: Cup is too heavy
- Replan: Ask for help or find alternative

### Multi-Step Reasoning

**Complex tasks:**
- LLM reasons through steps
- Considers dependencies
- Optimizes order

**Example:**
- Task: "Make breakfast"
- LLM thinks: Need ingredients → Cook → Serve
- Plans accordingly

## Integration with ROS 2

### Complete Pipeline

```
1. Voice Command → Whisper → Text
   ↓
2. Text → LLM → Plan (JSON)
   ↓
3. Plan → Executor → ROS 2 Actions
   ↓
4. Actions → Robot Controllers → Movement
   ↓
5. Feedback → LLM → Adjust Plan
```

### ROS 2 Action Integration

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class PlanExecutor(Node):
    def __init__(self):
        super().__init__('plan_executor')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def execute_step(self, step):
        if step['action'] == 'navigate':
            goal = NavigateToPose.Goal()
            # Set goal position from step
            self.nav_client.send_goal_async(goal)
```

## Best Practices

### 1. Validate Plans

**Before executing:**
- Check if plan is safe
- Verify steps are possible
- Confirm robot capabilities

### 2. Provide Context

**Give LLM information:**
- Current robot state
- Available objects
- Environment map
- Robot capabilities

### 3. Handle Failures

**If step fails:**
- Detect failure
- Ask LLM to replan
- Try alternative approach
- Ask human for help if needed

### 4. Monitor Progress

**Track execution:**
- Log each step
- Monitor success/failure
- Update LLM with status
- Adjust as needed

## Limitations and Considerations

### LLM Limitations

**Not perfect:**
- Can make mistakes
- May not understand context
- Might generate unsafe plans
- Can be slow

**Solutions:**
- Validate all plans
- Have safety checks
- Test thoroughly
- Use smaller models for speed

### Cost Considerations

**API costs:**
- Each call costs money
- Many calls add up
- Consider local models

**Alternatives:**
- Use local LLMs (Llama, etc.)
- Cache common plans
- Batch requests

## Real-World Example

### Complete System

**Components:**
1. Microphone → Records voice
2. Whisper → Converts to text
3. GPT-4 → Generates plan
4. Executor → Converts to ROS 2
5. Robot → Performs actions
6. Feedback → Updates status

**User says:** "Go to the kitchen, find a cup, and bring it to me"

**System:**
1. Whisper: "Go to the kitchen, find a cup, and bring it to me"
2. GPT-4: Generates 3-step plan
3. Executor: Converts to navigation + vision + manipulation
4. Robot: Executes each step
5. Success!

## Summary

- **AI Planning**: Use LLMs to understand complex commands
- **Process**: Command → LLM → Plan → Actions → Execution
- **Benefits**: Natural language, flexible, intelligent
- **Integration**: Works with ROS 2 and robot systems
- **Best Practice**: Validate plans, handle failures, monitor progress

## What's Next?

Now let's put it all together! Move to [Chapter 13: Putting It All Together](./chapter13) to see how vision, language, and action work together!





