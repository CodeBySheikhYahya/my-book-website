---
sidebar_position: 12
---

# Chapter 11: Teaching Robots to Understand Speech

## Introduction

Imagine talking to a robot like you talk to a person. "Go to the kitchen and bring me a cup." The robot understands and does it! This chapter shows you how to make robots understand human speech.

## Why Voice Commands?

### Natural Interaction

**Benefits:**
- **Intuitive**: Everyone knows how to talk
- **Hands-free**: Don't need to type or click
- **Fast**: Speaking is quicker than typing
- **Accessible**: Works for everyone

**Think about it:**
- You talk to Siri or Alexa
- Why not talk to robots the same way?
- Makes robots more user-friendly

## How Speech Recognition Works

### The Process

**Step 1: Capture Audio**
- Microphone records your voice
- Converts sound to digital signal
- Sends to computer

**Step 2: Process Audio**
- Remove background noise
- Enhance speech
- Prepare for recognition

**Step 3: Recognize Speech**
- AI model converts speech to text
- Identifies words
- Handles accents and variations

**Step 4: Understand Meaning**
- Parse the text
- Extract intent
- Understand what you want

**Step 5: Execute Action**
- Convert to robot commands
- Send to robot
- Robot acts!

## OpenAI Whisper: The Best Speech Recognition

### What is Whisper?

**Whisper** is OpenAI's speech recognition system:
- Very accurate
- Handles many languages
- Works with accents
- Free and open-source
- Easy to use

### Why Whisper?

**Advantages:**
- **Accuracy**: Very good at understanding speech
- **Robust**: Works in noisy environments
- **Multilingual**: Supports many languages
- **Open Source**: Free to use
- **Easy Integration**: Simple to add to robots

### How Accurate is Whisper?

- **English**: Over 95% accuracy
- **Other Languages**: Very good
- **Noisy Environments**: Still works well
- **Accents**: Handles variations

## Setting Up Whisper

### Installation

**Using Python:**
```bash
pip install openai-whisper
```

**Or using pip:**
```bash
pip install git+https://github.com/openai/whisper.git
```

### Basic Usage

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("audio.wav")
print(result["text"])
```

**That's it!** Very simple to use.

## Integrating Whisper with ROS 2

### Creating a Speech Recognition Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import wave

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        # Load Whisper model
        self.model = whisper.load_model("base")
        
        # Publisher for recognized text
        self.text_pub = self.create_publisher(
            String,
            '/speech/text',
            10
        )
        
        # Timer to process audio
        timer_period = 2.0  # Process every 2 seconds
        self.timer = self.create_timer(timer_period, self.process_audio)
        
    def process_audio(self):
        # Record audio (simplified - you'd use pyaudio)
        # audio_data = record_audio()
        
        # Transcribe with Whisper
        # result = self.model.transcribe(audio_data)
        # text = result["text"]
        
        # Publish text
        # msg = String()
        # msg.data = text
        # self.text_pub.publish(msg)
        
        self.get_logger().info('Processing audio...')

def main():
    rclpy.init()
    node = SpeechRecognitionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Recording Audio

### Using PyAudio

```python
import pyaudio
import wave

def record_audio(duration=5, sample_rate=16000):
    chunk = 1024
    format = pyaudio.paInt16
    channels = 1
    
    audio = pyaudio.PyAudio()
    
    stream = audio.open(
        format=format,
        channels=channels,
        rate=sample_rate,
        input=True,
        frames_per_buffer=chunk
    )
    
    frames = []
    for _ in range(0, int(sample_rate / chunk * duration)):
        data = stream.read(chunk)
        frames.append(data)
    
    stream.stop_stream()
    stream.close()
    audio.terminate()
    
    return b''.join(frames)
```

## Real-Time Speech Recognition

### Continuous Listening

**Challenge**: Process audio continuously
**Solution**: Use threading or async processing

```python
import threading

class ContinuousListener:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.listening = False
        
    def start_listening(self):
        self.listening = True
        thread = threading.Thread(target=self._listen_loop)
        thread.start()
        
    def _listen_loop(self):
        while self.listening:
            audio = record_audio(duration=3)
            text = self.model.transcribe(audio)["text"]
            if text.strip():
                self.process_command(text)
```

## Understanding Commands

### Natural Language Processing

**After getting text, you need to:**
1. Parse the command
2. Extract intent
3. Identify parameters
4. Convert to robot actions

**Example:**
- Input: "Go to the kitchen"
- Intent: Navigate
- Location: Kitchen
- Action: Move robot to kitchen

### Simple Command Parsing

```python
def parse_command(text):
    text_lower = text.lower()
    
    # Navigation commands
    if "go to" in text_lower or "move to" in text_lower:
        # Extract location
        location = extract_location(text_lower)
        return {"action": "navigate", "location": location}
    
    # Manipulation commands
    elif "pick up" in text_lower or "grab" in text_lower:
        object_name = extract_object(text_lower)
        return {"action": "pick", "object": object_name}
    
    # Stop commands
    elif "stop" in text_lower or "halt" in text_lower:
        return {"action": "stop"}
    
    return None
```

## Voice-to-Action Pipeline

### Complete Flow

```
1. Microphone captures audio
   ↓
2. Whisper converts to text
   ↓
3. NLP extracts intent and parameters
   ↓
4. Convert to ROS 2 commands
   ↓
5. Send to robot controllers
   ↓
6. Robot executes action
   ↓
7. (Optional) Robot confirms with voice
```

## Example: Voice-Controlled Robot

### Complete Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import whisper

class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_robot')
        self.model = whisper.load_model("base")
        
        # Publisher for movement
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscriber for recognized text
        self.text_sub = self.create_subscription(
            String,
            '/speech/text',
            self.handle_command,
            10
        )
        
    def handle_command(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f'Heard: {text}')
        
        cmd = Twist()
        
        if "forward" in text or "go ahead" in text:
            cmd.linear.x = 0.5
        elif "backward" in text or "go back" in text:
            cmd.linear.x = -0.5
        elif "left" in text:
            cmd.angular.z = 0.5
        elif "right" in text:
            cmd.angular.z = -0.5
        elif "stop" in text:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = VoiceControlledRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Errors

### Common Issues

**1. Background Noise**
- Use noise cancellation
- Better microphones
- Filter audio

**2. Misunderstanding**
- Confirm commands
- Ask for clarification
- Show what robot understood

**3. Multiple Commands**
- Queue commands
- Process one at a time
- Handle interruptions

## Best Practices

### 1. Use Good Microphones
- Directional microphones help
- Reduce background noise
- Better audio quality

### 2. Confirm Understanding
- Repeat back what robot heard
- Ask for confirmation if unclear
- Show confidence level

### 3. Handle Failures Gracefully
- If can't understand, ask again
- Don't execute uncertain commands
- Provide feedback

### 4. Optimize for Real-Time
- Use smaller Whisper models for speed
- Process in background
- Balance accuracy vs. speed

## Advanced Features

### Wake Word Detection

**Wake words**: "Hey Robot", "Robot", etc.
- Robot only listens after wake word
- Saves processing power
- More natural interaction

### Multi-Language Support

- Whisper supports many languages
- Detect language automatically
- Switch languages dynamically

### Voice Feedback

- Robot speaks back
- Confirms actions
- Provides status updates
- More interactive

## Summary

- **Voice Commands**: Natural way to control robots
- **Whisper**: Excellent speech recognition
- **Integration**: Connect to ROS 2 easily
- **Pipeline**: Audio → Text → Intent → Action
- **Best Practice**: Confirm understanding, handle errors gracefully

## What's Next?

Now let's learn how to use Large Language Models (LLMs) to make robots understand complex commands and plan actions! Move to [Chapter 12: AI Planning for Robots](./chapter12)!





