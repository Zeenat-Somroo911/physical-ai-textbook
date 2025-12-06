---
sidebar_position: 5
---

# Project 4: Voice-Controlled Robot Butler

## Project Overview

Build a complete Voice-Language-Action (VLA) system that enables a robot to understand natural language commands, perceive its environment, and execute complex tasks autonomously.

### Learning Objectives

- Integrate voice recognition with ROS 2
- Use LLMs for natural language understanding
- Implement action planning and execution
- Combine vision, language, and action
- Build a complete VLA system

### Prerequisites

- Completed [Module 4: Vision-Language-Action](../module-04-vla/vla-introduction)
- OpenAI API key ($5 credit)
- Understanding of multimodal systems
- Basic knowledge of action planning

## Project Requirements

### Functional Requirements

1. **Voice Recognition**: Understand spoken commands
2. **Language Understanding**: Parse and understand commands
3. **Action Planning**: Break down commands into actions
4. **Vision Integration**: Use camera for scene understanding
5. **Task Execution**: Execute planned actions

### Technical Requirements

- Web Speech API or Whisper for voice
- OpenAI API for language understanding
- ROS 2 for robot control
- Computer vision for perception
- Action planning system

## Step-by-Step Implementation

### Step 1: Create ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_butler \
    --dependencies rclpy std_msgs sensor_msgs geometry_msgs
cd ~/ros2_ws
colcon build --packages-select robot_butler
source install/setup.bash
```

### Step 2: Voice Recognition Node

Create `robot_butler/robot_butler/voice_recognizer.py`:

```python
#!/usr/bin/env python3
"""
Voice Recognition Node

Recognizes voice commands using Web Speech API or Whisper.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import threading

class VoiceRecognizerNode(Node):
    def __init__(self):
        super().__init__('voice_recognizer_node')
        
        # Speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        
        # Publisher
        self.command_pub = self.create_publisher(
            String,
            '/butler/voice_command',
            10
        )
        
        # Start listening thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()
        
        self.get_logger().info('Voice recognizer node started')
    
    def listen_loop(self):
        """Continuous listening loop."""
        while self.listening and rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(
                        source,
                        timeout=1,
                        phrase_time_limit=5
                    )
                
                try:
                    # Use Google Speech Recognition (free)
                    text = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f'Recognized: {text}')
                    
                    # Publish command
                    msg = String()
                    msg.data = text
                    self.command_pub.publish(msg)
                
                except sr.UnknownValueError:
                    pass
                except sr.RequestError as e:
                    self.get_logger().error(f'Recognition error: {e}')
            
            except sr.WaitTimeoutError:
                pass
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.listening = False
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: LLM Integration Node

Create `robot_butler/robot_butler/llm_integration.py`:

```python
#!/usr/bin/env python3
"""
LLM Integration Node

Processes commands using OpenAI API.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json

class LLMIntegrationNode(Node):
    def __init__(self):
        super().__init__('llm_integration_node')
        
        # Set OpenAI API key
        openai.api_key = os.getenv('OPENAI_API_KEY')
        if not openai.api_key:
            self.get_logger().error('OPENAI_API_KEY not set!')
        
        # Subscriber
        self.command_sub = self.create_subscription(
            String,
            '/butler/voice_command',
            self.command_callback,
            10
        )
        
        # Publisher
        self.plan_pub = self.create_publisher(
            String,
            '/butler/action_plan',
            10
        )
        
        self.get_logger().info('LLM integration node started')
    
    def command_callback(self, msg):
        """Process command using LLM."""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')
        
        # Generate action plan using LLM
        plan = self.generate_plan(command)
        
        if plan:
            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
    
    def generate_plan(self, command):
        """Generate action plan using OpenAI."""
        prompt = f"""You are a robot butler. Break down this command into actions: "{command}"

Available actions:
- navigate_to(location)
- pick_up(object)
- place(object, location)
- open(door)
- close(door)
- check(object)
- wait(duration)

Return JSON array:
[
    {{"action": "action_name", "parameters": {{...}}, "description": "..."}},
    ...
]"""
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot butler planner. Return valid JSON arrays."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )
            
            text = response.choices[0].message.content.strip()
            
            # Extract JSON
            if '```' in text:
                text = text.split('```')[1]
                if text.startswith('json'):
                    text = text[4:]
            
            plan = json.loads(text)
            self.get_logger().info(f'Plan generated: {len(plan)} steps')
            return plan
        
        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = LLMIntegrationNode()
    
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

### Step 4: Vision Processor Node

Create `robot_butler/robot_butler/vision_processor.py`:

```python
#!/usr/bin/env python3
"""
Vision Processor Node

Processes camera images for scene understanding.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')
        
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher
        self.scene_pub = self.create_publisher(
            String,
            '/butler/scene',
            10
        )
        
        self.detected_objects = []
        
        self.get_logger().info('Vision processor node started')
    
    def image_callback(self, msg):
        """Process images for scene understanding."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Downscale for speed
            small_image = cv2.resize(cv_image, (320, 240))
            
            # Detect objects
            results = self.yolo_model(small_image)
            
            self.detected_objects = []
            for result in results:
                for box in result.boxes:
                    if float(box.conf[0]) > 0.5:
                        self.detected_objects.append({
                            'label': self.yolo_model.names[int(box.cls[0])],
                            'confidence': float(box.conf[0]),
                            'bbox': box.xyxy[0].cpu().numpy().tolist()
                        })
            
            # Publish scene description
            scene_msg = String()
            scene_msg.data = json.dumps(self.detected_objects)
            self.scene_pub.publish(scene_msg)
        
        except Exception as e:
            self.get_logger().error(f'Vision error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()
    
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

### Step 5: Action Executor Node

Create `robot_butler/robot_butler/action_executor.py`:

```python
#!/usr/bin/env python3
"""
Action Executor Node

Executes planned actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import math

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')
        
        # Subscriber
        self.plan_sub = self.create_subscription(
            String,
            '/butler/action_plan',
            self.plan_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/butler/status', 10)
        
        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.current_plan = []
        self.current_step = 0
        self.executing = False
        
        # Timer
        self.timer = self.create_timer(0.1, self.execution_loop)
        
        self.get_logger().info('Action executor node started')
    
    def plan_callback(self, msg):
        """Receive new plan."""
        try:
            plan = json.loads(msg.data)
            self.current_plan = plan
            self.current_step = 0
            self.executing = True
            self.get_logger().info(f'New plan received: {len(plan)} steps')
        except Exception as e:
            self.get_logger().error(f'Plan parsing error: {e}')
    
    def execution_loop(self):
        """Main execution loop."""
        if self.executing and self.current_step < len(self.current_plan):
            action = self.current_plan[self.current_step]
            self.execute_action(action)
    
    def execute_action(self, action):
        """Execute a single action."""
        action_name = action.get('action')
        params = action.get('parameters', {})
        description = action.get('description', '')
        
        self.get_logger().info(f'Step {self.current_step + 1}: {description}')
        
        success = False
        
        if action_name == 'navigate_to':
            success = self.navigate_to(params)
        elif action_name == 'pick_up':
            success = self.pick_up(params)
        elif action_name == 'place':
            success = self.place(params)
        elif action_name == 'stop':
            success = self.stop()
        else:
            self.get_logger().warn(f'Unknown action: {action_name}')
        
        if success:
            self.current_step += 1
            
            if self.current_step >= len(self.current_plan):
                self.executing = False
                self.get_logger().info('Plan completed!')
                
                status_msg = String()
                status_msg.data = 'completed'
                self.status_pub.publish(status_msg)
    
    def navigate_to(self, params):
        """Navigate to location."""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        theta = params.get('theta', 0.0)
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        return goal_handle.accepted
    
    def pick_up(self, params):
        """Pick up object."""
        object_name = params.get('object', 'unknown')
        self.get_logger().info(f'Picking up {object_name}')
        return True
    
    def place(self, params):
        """Place object."""
        object_name = params.get('object', 'unknown')
        location = params.get('location', 'unknown')
        self.get_logger().info(f'Placing {object_name} at {location}')
        return True
    
    def stop(self):
        """Stop robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    
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

### Step 6: Complete Launch File

Create `robot_butler/launch/butler.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Voice recognizer
        Node(
            package='robot_butler',
            executable='voice_recognizer',
            name='voice_recognizer',
            output='screen'
        ),
        
        # LLM integration
        Node(
            package='robot_butler',
            executable='llm_integration',
            name='llm_integration',
            output='screen'
        ),
        
        # Vision processor
        Node(
            package='robot_butler',
            executable='vision_processor',
            name='vision_processor',
            output='screen'
        ),
        
        # Action executor
        Node(
            package='robot_butler',
            executable='action_executor',
            name='action_executor',
            output='screen'
        ),
    ])
```

## Testing Steps

### Test 1: Voice Recognition

```bash
# Start voice recognizer
ros2 run robot_butler voice_recognizer

# Speak commands
# Check output
ros2 topic echo /butler/voice_command
```

**Expected Output**: Spoken commands converted to text.

### Test 2: LLM Planning

```bash
# Set API key
export OPENAI_API_KEY='your-key-here'

# Start LLM node
ros2 run robot_butler llm_integration

# Send test command
ros2 topic pub /butler/voice_command std_msgs/msg/String "{data: 'pick up the cup'}"

# Check plan
ros2 topic echo /butler/action_plan
```

**Expected Output**: Action plan generated from command.

### Test 3: Vision Processing

```bash
# Start vision processor
ros2 run robot_butler vision_processor

# Check scene
ros2 topic echo /butler/scene
```

**Expected Output**: Objects detected and scene description published.

### Test 4: Complete System

```bash
# Launch all components
ros2 launch robot_butler butler.launch.py

# Speak: "Go to the kitchen and pick up the cup"
# Watch robot execute the plan
```

**Expected Output**: Robot understands command, plans actions, and executes them.

## Expected Outputs

### Voice Recognition

- Commands recognized and published
- Text output matches speech

### LLM Planning

- Commands broken down into actions
- Valid JSON plan generated
- Actions are executable

### Vision Processing

- Objects detected in scene
- Scene description available
- Real-time updates

### Action Execution

- Actions executed in sequence
- Status updates published
- Tasks completed successfully

## Grading Rubric

### Voice Recognition (20 points)

- **Recognition Accuracy** (10 points): Commands recognized correctly
- **ROS Integration** (10 points): Commands published to topics

### LLM Integration (25 points)

- **Plan Generation** (15 points): Valid plans generated
- **Command Understanding** (10 points): Commands understood correctly

### Vision Processing (20 points)

- **Object Detection** (10 points): Objects detected
- **Scene Understanding** (10 points): Scene described accurately

### Action Execution (25 points)

- **Action Execution** (15 points): Actions executed correctly
- **Task Completion** (10 points): Tasks completed successfully

### Integration (10 points)

- **System Integration** (5 points): All components working together
- **Performance** (5 points): Real-time operation

## Extensions

### Extension 1: Multimodal Understanding

- Combine vision and language
- Context-aware planning
- Better object grounding

### Extension 2: Learning

- Learn from demonstrations
- Improve with experience
- Adapt to new scenarios

### Extension 3: Advanced Tasks

- Complex multi-step tasks
- Task prioritization
- Error recovery

## Troubleshooting

### Issue: Voice not recognized

**Solution**: Check microphone, reduce noise, speak clearly.

### Issue: LLM not generating plans

**Solution**: Check API key, verify internet connection, check prompt format.

### Issue: Actions not executing

**Solution**: Verify action executor, check robot hardware, verify topics.

## Next Steps

- [Final Project: Capstone Guidelines](final-project.md) - Complete capstone
- [Module 4: Capstone Project](../module-04-vla/capstone-project) - Detailed capstone guide

---

**Amazing work!** You've built a complete voice-controlled robot butler! 🎉

