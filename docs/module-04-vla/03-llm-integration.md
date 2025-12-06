---
sidebar_position: 3
---

# Chapter 3: LLM Integration

## OpenAI API Setup ($5 Free Credit)

OpenAI provides $5 in free credit when you sign up, which is more than enough to learn and prototype VLA systems.

### Getting Started

1. **Sign Up**: Go to https://platform.openai.com/signup
2. **Get API Key**: Navigate to API Keys section
3. **Add Payment**: Add $5 (or use free credit)
4. **Set Up**: Install library and configure

### Installation

```bash
# Install OpenAI library
pip install openai

# Set API key
export OPENAI_API_KEY="sk-your-key-here"

# Or in Python
import os
os.environ['OPENAI_API_KEY'] = 'sk-your-key-here'
```

### Cost Breakdown

```
GPT-3.5-turbo:    $0.002 per 1K tokens (input)
                  $0.002 per 1K tokens (output)

With $5 credit:
- ~2.5M input tokens
- ~2.5M output tokens
- Enough for 1000+ conversations
```

## Prompt Engineering for Robots

Effective prompts are crucial for getting robots to understand and execute commands correctly.

### Basic Prompt Structure

```python
import openai

def create_robot_prompt(user_command, robot_state):
    """Create prompt for robot command."""
    prompt = f"""You are a robot assistant. Convert the user's natural language command into robot actions.

Current robot state:
- Position: {robot_state['position']}
- Battery: {robot_state['battery']}%
- Status: {robot_state['status']}

User command: {user_command}

Respond with a JSON object containing:
- action: The action to perform (move, pick, place, etc.)
- parameters: Action parameters (direction, distance, object, etc.)
- reasoning: Brief explanation

Example response:
{{
    "action": "move",
    "parameters": {{"direction": "forward", "distance": 1.0}},
    "reasoning": "User wants to move forward 1 meter"
}}
"""
    return prompt
```

### Advanced Prompting Techniques

#### 1. Few-Shot Learning

```python
def create_few_shot_prompt(command):
    """Prompt with examples."""
    prompt = f"""Convert natural language to robot actions.

Examples:
1. User: "Pick up the cup"
   Response: {{"action": "pick", "object": "cup", "location": "detect"}}

2. User: "Move forward 2 meters"
   Response: {{"action": "move", "direction": "forward", "distance": 2.0}}

3. User: "Turn left and stop"
   Response: {{"action": "turn", "direction": "left", "angle": 90}}, {{"action": "stop"}}

Now convert:
User: "{command}"
Response:"""
    return prompt
```

#### 2. Chain of Thought

```python
def create_cot_prompt(command):
    """Prompt with reasoning steps."""
    prompt = f"""Convert the command to robot actions. Think step by step.

Command: {command}

Step 1: Understand what the user wants
Step 2: Identify required actions
Step 3: Determine parameters
Step 4: Generate action sequence

Response format:
{{
    "steps": ["step1", "step2", ...],
    "actions": [{{"action": "...", "params": {{...}}}}, ...]
}}"""
    return prompt
```

#### 3. Constrained Output

```python
def create_constrained_prompt(command):
    """Prompt with output constraints."""
    prompt = f"""Convert command to robot action. Use ONLY these actions:
- move(direction, distance)
- turn(direction, angle)
- pick(object)
- place(object, location)
- stop()

Command: {command}

Response (JSON only):"""
    return prompt
```

## Converting Language to Actions

### Action Parser

```python
#!/usr/bin/env python3
"""
Language to Action Converter

Converts natural language to robot actions using OpenAI.
"""

import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class LanguageToAction(Node):
    def __init__(self):
        super().__init__('language_to_action')
        
        # Set API key
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Subscriber
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Robot state
        self.robot_state = {
            'position': [0, 0, 0],
            'battery': 100,
            'status': 'idle'
        }
        
        self.get_logger().info('Language to action converter started')
    
    def command_callback(self, msg):
        """Process voice command."""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')
        
        # Convert to action
        action = self.convert_to_action(command)
        
        if action:
            self.execute_action(action)
    
    def convert_to_action(self, command):
        """Convert natural language to action using GPT."""
        prompt = f"""You are a robot controller. Convert the user's command into a robot action.

Robot state:
- Position: {self.robot_state['position']}
- Battery: {self.robot_state['battery']}%
- Status: {self.robot_state['status']}

User command: "{command}"

Respond with JSON only:
{{
    "action": "move|turn|pick|place|stop",
    "parameters": {{...}},
    "confidence": 0.0-1.0
}}

Examples:
- "move forward" -> {{"action": "move", "parameters": {{"direction": "forward", "distance": 1.0}}, "confidence": 0.9}}
- "turn left 90 degrees" -> {{"action": "turn", "parameters": {{"direction": "left", "angle": 90}}, "confidence": 0.9}}
- "stop" -> {{"action": "stop", "parameters": {{}}, "confidence": 1.0}}
"""
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",  # Cheapest option
                messages=[
                    {"role": "system", "content": "You are a robot action parser. Always respond with valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower = more deterministic
                max_tokens=150
            )
            
            # Parse response
            text = response.choices[0].message.content.strip()
            
            # Extract JSON (handle markdown code blocks)
            if '```' in text:
                text = text.split('```')[1]
                if text.startswith('json'):
                    text = text[4:]
            
            action = json.loads(text)
            return action
        
        except Exception as e:
            self.get_logger().error(f'Error converting command: {e}')
            return None
    
    def execute_action(self, action):
        """Execute robot action."""
        action_type = action.get('action')
        params = action.get('parameters', {})
        
        if action_type == 'move':
            self.execute_move(params)
        elif action_type == 'turn':
            self.execute_turn(params)
        elif action_type == 'stop':
            self.execute_stop()
        else:
            self.get_logger().warn(f'Unknown action: {action_type}')
    
    def execute_move(self, params):
        """Execute move action."""
        direction = params.get('direction', 'forward')
        distance = params.get('distance', 1.0)
        
        cmd = Twist()
        if direction == 'forward':
            cmd.linear.x = 0.5
        elif direction == 'backward':
            cmd.linear.x = -0.5
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Moving {direction} {distance}m')
    
    def execute_turn(self, params):
        """Execute turn action."""
        direction = params.get('direction', 'left')
        angle = params.get('angle', 90)
        
        cmd = Twist()
        if direction == 'left':
            cmd.angular.z = 0.5
        elif direction == 'right':
            cmd.angular.z = -0.5
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Turning {direction} {angle} degrees')
    
    def execute_stop(self):
        """Execute stop action."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Stopping')

def main(args=None):
    rclpy.init(args=args)
    node = LanguageToAction()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Function Calling

Function calling allows GPT to call predefined functions, making it more reliable for robot control.

### Setup Function Calling

```python
#!/usr/bin/env python3
"""
Function Calling for Robot Control

Use OpenAI function calling for structured robot commands.
"""

import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class FunctionCallingRobot(Node):
    def __init__(self):
        super().__init__('function_calling_robot')
        
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Subscriber
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define functions
        self.functions = [
            {
                "name": "move_robot",
                "description": "Move the robot in a direction",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["forward", "backward", "left", "right"],
                            "description": "Direction to move"
                        },
                        "distance": {
                            "type": "number",
                            "description": "Distance in meters"
                        }
                    },
                    "required": ["direction"]
                }
            },
            {
                "name": "turn_robot",
                "description": "Turn the robot",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Turn direction"
                        },
                        "angle": {
                            "type": "number",
                            "description": "Angle in degrees"
                        }
                    },
                    "required": ["direction"]
                }
            },
            {
                "name": "stop_robot",
                "description": "Stop the robot",
                "parameters": {
                    "type": "object",
                    "properties": {}
                }
            }
        ]
        
        self.get_logger().info('Function calling robot started')
    
    def command_callback(self, msg):
        """Process command with function calling."""
        command = msg.data
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot controller. Use the provided functions to control the robot."},
                    {"role": "user", "content": command}
                ],
                functions=self.functions,
                function_call="auto"
            )
            
            message = response.choices[0].message
            
            # Check if function was called
            if message.get("function_call"):
                function_name = message["function_call"]["name"]
                function_args = json.loads(message["function_call"]["arguments"])
                
                # Execute function
                self.execute_function(function_name, function_args)
            else:
                self.get_logger().info(f'Response: {message["content"]}')
        
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def execute_function(self, function_name, args):
        """Execute called function."""
        if function_name == "move_robot":
            self.move_robot(args)
        elif function_name == "turn_robot":
            self.turn_robot(args)
        elif function_name == "stop_robot":
            self.stop_robot()
    
    def move_robot(self, args):
        """Move robot."""
        direction = args.get('direction')
        distance = args.get('distance', 1.0)
        
        cmd = Twist()
        if direction == 'forward':
            cmd.linear.x = 0.5
        elif direction == 'backward':
            cmd.linear.x = -0.5
        elif direction == 'left':
            cmd.angular.z = 0.5
        elif direction == 'right':
            cmd.angular.z = -0.5
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Moving {direction} {distance}m')
    
    def turn_robot(self, args):
        """Turn robot."""
        direction = args.get('direction')
        angle = args.get('angle', 90)
        
        cmd = Twist()
        if direction == 'left':
            cmd.angular.z = 0.5
        else:
            cmd.angular.z = -0.5
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Turning {direction} {angle} degrees')
    
    def stop_robot(self):
        """Stop robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Stopping')

def main(args=None):
    rclpy.init(args=args)
    node = FunctionCallingRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Working System

### Full VLA System with LLM

```python
#!/usr/bin/env python3
"""
Complete VLA System with LLM

Integrates voice, vision, and LLM for complete VLA.
"""

import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import base64
import io

class CompleteVLASystem(Node):
    def __init__(self):
        super().__init__('complete_vla_system')
        
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        # Components
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')
        
        # Subscribers
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.voice_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.current_image = None
        self.detected_objects = []
        
        self.get_logger().info('Complete VLA system started')
    
    def image_callback(self, msg):
        """Process images."""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect objects
            results = self.yolo_model(self.current_image)
            self.detected_objects = []
            
            for result in results:
                for box in result.boxes:
                    if float(box.conf[0]) > 0.5:
                        self.detected_objects.append({
                            'label': self.yolo_model.names[int(box.cls[0])],
                            'confidence': float(box.conf[0])
                        })
        
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def voice_callback(self, msg):
        """Process voice commands with vision context."""
        command = msg.data
        self.get_logger().info(f'Processing: {command}')
        
        # Create prompt with vision context
        prompt = self.create_multimodal_prompt(command)
        
        # Get LLM response
        action = self.get_llm_action(prompt)
        
        if action:
            self.execute_action(action)
    
    def create_multimodal_prompt(self, command):
        """Create prompt with vision and language."""
        objects_str = ', '.join([obj['label'] for obj in self.detected_objects])
        
        prompt = f"""You are a robot assistant. The user said: "{command}"

Current scene contains: {objects_str if objects_str else "no detected objects"}

Convert this to a robot action. Available actions:
- move(direction, distance)
- turn(direction, angle)
- pick(object)
- place(object, location)
- stop()

Respond with JSON:
{{
    "action": "...",
    "parameters": {{...}},
    "reasoning": "..."
}}"""
        return prompt
    
    def get_llm_action(self, prompt):
        """Get action from LLM."""
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot controller. Always respond with valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=200
            )
            
            text = response.choices[0].message.content.strip()
            
            # Parse JSON
            if '```' in text:
                text = text.split('```')[1]
                if text.startswith('json'):
                    text = text[4:]
            
            return json.loads(text)
        
        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return None
    
    def execute_action(self, action):
        """Execute action."""
        action_type = action.get('action')
        params = action.get('parameters', {})
        
        self.get_logger().info(f'Executing: {action_type} with {params}')
        
        # Execute based on action type
        # (Implementation similar to previous examples)

def main(args=None):
    rclpy.init(args=args)
    node = CompleteVLASystem()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Cost Optimization Tips

### 1. Use GPT-3.5-turbo

```python
# Cheapest model
model = "gpt-3.5-turbo"  # $0.002 per 1K tokens
# Instead of
# model = "gpt-4"  # $0.03 per 1K tokens (15x more expensive)
```

### 2. Cache Common Queries

```python
# Cache responses
cache = {}

def get_cached_response(prompt):
    if prompt in cache:
        return cache[prompt]
    
    response = openai.ChatCompletion.create(...)
    cache[prompt] = response
    return response
```

### 3. Reduce Token Usage

```python
# Use shorter prompts
prompt = f"Command: {command}\nAction:"  # Short
# Instead of
# prompt = f"Long detailed explanation...{command}..."  # Long
```

### 4. Batch Requests

```python
# Process multiple commands at once
commands = ["move forward", "turn left", "stop"]
batch_prompt = "\n".join([f"Command {i+1}: {cmd}" for i, cmd in enumerate(commands)])
```

### 5. Monitor Usage

```python
# Track API usage
def track_usage(response):
    tokens_used = response.usage.total_tokens
    cost = tokens_used * 0.002 / 1000  # GPT-3.5-turbo pricing
    print(f"Tokens: {tokens_used}, Cost: ${cost:.4f}")
```

## Performance Benchmarks

### Response Times

| Model | Latency | Cost per 1K tokens |
|-------|---------|-------------------|
| GPT-3.5-turbo | 200-500ms | $0.002 |
| GPT-4 | 500-2000ms | $0.03 |
| GPT-4-turbo | 300-1000ms | $0.01 |

### Cost per Session

```
Average conversation:
- Input: ~100 tokens
- Output: ~50 tokens
- Cost: ~$0.0003 per conversation

With $5 credit: ~16,000 conversations
```

## Best Practices

1. **Use function calling**: More reliable than JSON parsing
2. **Set temperature low**: 0.3 for deterministic responses
3. **Validate outputs**: Always check LLM responses
4. **Handle errors**: Graceful fallback for API failures
5. **Monitor costs**: Track usage to stay within budget

## Common Errors and Solutions

### Error 1: "API key not set"

```python
# Solution
import os
os.environ['OPENAI_API_KEY'] = 'sk-your-key'
```

### Error 2: "Rate limit exceeded"

```python
# Solution: Add retry logic
import time

def call_with_retry(prompt, max_retries=3):
    for i in range(max_retries):
        try:
            return openai.ChatCompletion.create(...)
        except openai.error.RateLimitError:
            time.sleep(2 ** i)  # Exponential backoff
```

### Error 3: "Invalid JSON response"

```python
# Solution: Use function calling or add JSON extraction
import re

def extract_json(text):
    # Try to find JSON in response
    json_match = re.search(r'\{.*\}', text, re.DOTALL)
    if json_match:
        return json.loads(json_match.group())
    return None
```

## Next Steps

Continue learning:
- [Chapter 4: Action Planning](04-action-planning.md) - Plan complex tasks
- [Chapter 5: Multimodal Systems](05-multimodal.md) - Complete pipeline

