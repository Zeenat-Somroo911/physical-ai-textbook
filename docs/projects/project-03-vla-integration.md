---
sidebar_position: 4
---

# Project 3: VLA Integration

## Objective

Integrate a Vision-Language-Action model with a ROS2 robot system to enable natural language control.

## Prerequisites

- Completion of all modules
- PyTorch installed
- Pre-trained VLA model (or ability to train one)
- ROS2 setup

## Project Overview

This project creates a ROS2 node that:
1. Receives camera images
2. Processes natural language instructions
3. Uses VLA model to predict actions
4. Publishes robot commands

## Step 1: VLA Model Wrapper

Create `vla_integration/nodes/vla_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
from transformers import AutoTokenizer, AutoModel

class VLARobotNode(Node):
    def __init__(self):
        super().__init__('vla_robot_node')
        
        # Initialize VLA model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.load_vla_model()
        
        # Image processing
        self.bridge = CvBridge()
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225])
        ])
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.instruction_sub = self.create_subscription(
            String,
            '/instruction',
            self.instruction_callback,
            10
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.current_image = None
        self.current_instruction = None
        
        self.get_logger().info('VLA Robot Node started')
    
    def load_vla_model(self):
        """Load pre-trained VLA model"""
        # This is a placeholder - replace with your actual model
        self.get_logger().info('Loading VLA model...')
        # self.model = YourVLAModel()
        # self.model.eval()
        self.get_logger().info('VLA model loaded')
    
    def image_callback(self, msg):
        """Store latest image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.current_image = cv_image
            self.process_if_ready()
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def instruction_callback(self, msg):
        """Store latest instruction"""
        self.current_instruction = msg.data
        self.get_logger().info(f'Received instruction: {self.current_instruction}')
        self.process_if_ready()
    
    def process_if_ready(self):
        """Process when both image and instruction are available"""
        if self.current_image is not None and self.current_instruction is not None:
            action = self.predict_action(self.current_image, self.current_instruction)
            self.publish_action(action)
    
    def predict_action(self, image, instruction):
        """Use VLA model to predict action"""
        # Preprocess image
        image_tensor = self.transform(image).unsqueeze(0).to(self.device)
        
        # Tokenize instruction
        # tokens = self.tokenizer(instruction, return_tensors='pt').to(self.device)
        
        # Predict action (placeholder)
        # with torch.no_grad():
        #     action = self.model(image_tensor, tokens)
        
        # For now, return a simple action based on keywords
        action = self.simple_action_parser(instruction)
        return action
    
    def simple_action_parser(self, instruction):
        """Simple keyword-based action parser (replace with actual VLA model)"""
        instruction_lower = instruction.lower()
        
        # Parse movement commands
        linear_x = 0.0
        angular_z = 0.0
        
        if 'forward' in instruction_lower or 'ahead' in instruction_lower:
            linear_x = 0.5
        elif 'backward' in instruction_lower or 'back' in instruction_lower:
            linear_x = -0.5
        elif 'stop' in instruction_lower or 'halt' in instruction_lower:
            linear_x = 0.0
            angular_z = 0.0
        
        if 'left' in instruction_lower:
            angular_z = 0.5
        elif 'right' in instruction_lower:
            angular_z = -0.5
        
        return {'linear_x': linear_x, 'angular_z': angular_z}
    
    def publish_action(self, action):
        """Publish robot command"""
        cmd = Twist()
        cmd.linear.x = action.get('linear_x', 0.0)
        cmd.angular.z = action.get('angular_z', 0.0)
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Publishing command: linear={cmd.linear.x}, angular={cmd.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VLARobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 2: Integration with Simulation

Update launch file to include VLA node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ... existing simulation launch ...
        
        # VLA Node
        Node(
            package='vla_integration',
            executable='vla_node',
            name='vla_robot_node',
            output='screen'
        )
    ])
```

## Step 3: Testing

```bash
# Launch simulation with VLA node
ros2 launch vla_integration complete_system.launch.py

# Send instruction
ros2 topic pub /instruction std_msgs/msg/String "data: 'move forward'"

# Monitor commands
ros2 topic echo /cmd_vel
```

## Step 4: Advanced VLA Integration

For a real VLA model, you would:

1. **Load Pre-trained Model**:
```python
from transformers import AutoModel, AutoTokenizer

model = AutoModel.from_pretrained('your-vla-model')
tokenizer = AutoTokenizer.from_pretrained('your-vla-model')
```

2. **Process Multimodal Input**:
```python
# Encode image and text
image_features = vision_encoder(image)
text_features = language_encoder(instruction)
fused_features = multimodal_fusion(image_features, text_features)
action = action_decoder(fused_features)
```

3. **Handle Action Space**:
```python
# Convert model output to robot commands
def action_to_twist(action_tensor):
    # Map model output to Twist message
    twist = Twist()
    twist.linear.x = action_tensor[0].item()
    twist.angular.z = action_tensor[1].item()
    return twist
```

## Safety Considerations

```python
class SafeVLANode(VLARobotNode):
    def __init__(self):
        super().__init__()
        self.max_velocity = 1.0
        self.max_angular = 1.0
    
    def publish_action(self, action):
        # Constrain actions for safety
        linear_x = np.clip(action['linear_x'], -self.max_velocity, self.max_velocity)
        angular_z = np.clip(action['angular_z'], -self.max_angular, self.max_angular)
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
```

## Testing Checklist

- [ ] VLA node receives images
- [ ] Instructions are processed correctly
- [ ] Actions are predicted
- [ ] Commands are published
- [ ] Robot responds appropriately
- [ ] Safety constraints are enforced

## Extensions

- Integrate with real VLA model (RT-1, RT-2, etc.)
- Add action history for temporal reasoning
- Implement feedback mechanism
- Add confidence scoring
- Create web interface for instructions

## Conclusion

You've successfully integrated a VLA system with ROS2! This enables natural language control of robots, opening up many possibilities for human-robot interaction.

## Next Steps

- Experiment with different VLA architectures
- Collect real-world data for fine-tuning
- Deploy on physical robot
- Explore multi-modal instructions (voice + text)

