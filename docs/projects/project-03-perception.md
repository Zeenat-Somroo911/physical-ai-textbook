---
sidebar_position: 4
---

# Project 3: Autonomous Navigation System

## Project Overview

Build a complete autonomous navigation system that combines object detection, SLAM, and Nav2 for a robot to navigate while avoiding obstacles and reaching goals.

### Learning Objectives

- Integrate computer vision with ROS 2
- Implement SLAM for map building
- Use Nav2 for path planning
- Combine perception and navigation
- Build a complete autonomous system

### Prerequisites

- Completed [Module 3: AI-Powered Perception](../module-03-isaac/free-alternatives)
- Understanding of OpenCV
- Basic knowledge of SLAM
- Nav2 installed

## Project Requirements

### Functional Requirements

1. **Object Detection**: Detect and classify objects in environment
2. **SLAM**: Build map while localizing
3. **Navigation**: Navigate to goals using Nav2
4. **Obstacle Avoidance**: Avoid detected objects
5. **Integration**: All components working together

### Technical Requirements

- YOLO for object detection
- RTAB-Map or ORB-SLAM3 for SLAM
- Nav2 stack for navigation
- ROS 2 integration
- Real-time processing

## Step-by-Step Implementation

### Step 1: Create ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python autonomous_nav \
    --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge
cd ~/ros2_ws
colcon build --packages-select autonomous_nav
source install/setup.bash
```

### Step 2: Object Detection Node

Create `autonomous_nav/autonomous_nav/object_detector.py`:

```python
#!/usr/bin/env python3
"""
Object Detection Node

Detects objects using YOLO and publishes detections.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import cv2

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/detections',
            10
        )
        
        self.annotated_pub = self.create_publisher(
            Image,
            '/detections/image_annotated',
            10
        )
        
        self.get_logger().info('Object detector node started')
    
    def image_callback(self, msg):
        """Process images and detect objects."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO detection
            results = self.yolo_model(cv_image)
            
            detections = []
            annotated_image = cv_image.copy()
            
            for result in results:
                for box in result.boxes:
                    if float(box.conf[0]) > 0.5:
                        label = self.yolo_model.names[int(box.cls[0])]
                        confidence = float(box.conf[0])
                        bbox = box.xyxy[0].cpu().numpy().tolist()
                        
                        detections.append({
                            'label': label,
                            'confidence': confidence,
                            'bbox': bbox
                        })
                        
                        # Draw bounding box
                        x1, y1, x2, y2 = map(int, bbox)
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            annotated_image,
                            f'{label}: {confidence:.2f}',
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )
            
            # Publish detections
            detection_msg = String()
            detection_msg.data = json.dumps(detections)
            self.detection_pub.publish(detection_msg)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
            if detections:
                self.get_logger().info(f'Detected {len(detections)} objects')
        
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    
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

### Step 3: SLAM Node

Create `autonomous_nav/autonomous_nav/slam_node.py`:

```python
#!/usr/bin/env python3
"""
SLAM Node

Simplified SLAM using visual odometry and map building.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from cv_bridge import CvBridge
import cv2

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/slam_pose',
            10
        )
        
        # Map initialization
        self.map_resolution = 0.05  # 5cm per pixel
        self.map_width = 400
        self.map_height = 400
        self.map_origin_x = -10.0
        self.map_origin_y = -10.0
        
        self.map_data = np.full(
            (self.map_height, self.map_width),
            -1,
            dtype=np.int8
        )
        
        # Visual odometry
        self.prev_image = None
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        
        # Timer for map publishing
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('SLAM node started')
    
    def image_callback(self, msg):
        """Process images for visual odometry."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            if self.prev_image is not None:
                # Feature detection and matching
                orb = cv2.ORB_create()
                kp1, des1 = orb.detectAndCompute(self.prev_image, None)
                kp2, des2 = orb.detectAndCompute(gray, None)
                
                if des1 is not None and des2 is not None:
                    # Match features
                    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                    matches = bf.match(des1, des2)
                    matches = sorted(matches, key=lambda x: x.distance)
                    
                    if len(matches) > 10:
                        # Estimate motion (simplified)
                        # In real implementation, use proper visual odometry
                        dx = 0.1  # Simplified
                        dy = 0.0
                        dtheta = 0.0
                        
                        # Update pose
                        self.robot_pose[0] += dx * np.cos(self.robot_pose[2]) - dy * np.sin(self.robot_pose[2])
                        self.robot_pose[1] += dx * np.sin(self.robot_pose[2]) + dy * np.cos(self.robot_pose[2])
                        self.robot_pose[2] += dtheta
                        
                        # Update map (simplified - mark current position as free)
                        map_x = int((self.robot_pose[0] - self.map_origin_x) / self.map_resolution)
                        map_y = int((self.robot_pose[1] - self.map_origin_y) / self.map_resolution)
                        
                        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                            self.map_data[map_y, map_x] = 0  # Free space
            
            self.prev_image = gray
            
            # Publish pose
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = self.robot_pose[0]
            pose_msg.pose.pose.position.y = self.robot_pose[1]
            pose_msg.pose.pose.orientation.z = np.sin(self.robot_pose[2] / 2.0)
            pose_msg.pose.pose.orientation.w = np.cos(self.robot_pose[2] / 2.0)
            self.pose_pub.publish(pose_msg)
        
        except Exception as e:
            self.get_logger().error(f'SLAM error: {e}')
    
    def publish_map(self):
        """Publish occupancy grid map."""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    
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

### Step 4: Navigation Integration Node

Create `autonomous_nav/autonomous_nav/navigation_integration.py`:

```python
#!/usr/bin/env python3
"""
Navigation Integration Node

Integrates object detection, SLAM, and Nav2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import math

class NavigationIntegrationNode(Node):
    def __init__(self):
        super().__init__('navigation_integration_node')
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            String,
            '/detections',
            self.detection_callback,
            10
        )
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.detected_objects = []
        self.current_goal = None
        
        # Timer for goal management
        self.timer = self.create_timer(1.0, self.manage_navigation)
        
        self.get_logger().info('Navigation integration node started')
    
    def detection_callback(self, msg):
        """Handle object detections."""
        try:
            self.detected_objects = json.loads(msg.data)
            
            # Check for obstacles in path
            for obj in self.detected_objects:
                if obj['label'] in ['person', 'car', 'bicycle']:
                    self.get_logger().warn(f'Obstacle detected: {obj["label"]}')
                    # In real implementation, replan path
        
        except Exception as e:
            self.get_logger().error(f'Detection callback error: {e}')
    
    def navigate_to_goal(self, x, y, theta=0.0):
        """Navigate to goal position."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            self.get_logger().info(f'Navigating to ({x}, {y})')
            return True
        else:
            self.get_logger().error('Goal rejected')
            return False
    
    def manage_navigation(self):
        """Manage navigation goals."""
        # Example: Navigate to waypoints
        if self.current_goal is None:
            # Set first goal
            self.navigate_to_goal(2.0, 2.0, 0.0)
            self.current_goal = (2.0, 2.0)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationIntegrationNode()
    
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

### Step 5: Launch File

Create `autonomous_nav/launch/autonomous_nav.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Object detector
        Node(
            package='autonomous_nav',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        
        # SLAM node
        Node(
            package='autonomous_nav',
            executable='slam_node',
            name='slam_node',
            output='screen'
        ),
        
        # Navigation integration
        Node(
            package='autonomous_nav',
            executable='navigation_integration',
            name='navigation_integration',
            output='screen'
        ),
    ])
```

## Testing Steps

### Test 1: Object Detection

```bash
# Start object detector
ros2 run autonomous_nav object_detector

# Check detections
ros2 topic echo /detections

# View annotated images
ros2 run rviz2 rviz2
# Add Image display for /detections/image_annotated
```

**Expected Output**: Objects detected and annotated in images.

### Test 2: SLAM

```bash
# Start SLAM node
ros2 run autonomous_nav slam_node

# Check map
ros2 topic echo /map

# Check pose
ros2 topic echo /slam_pose
```

**Expected Output**: Map building, pose estimates published.

### Test 3: Navigation

```bash
# Start Nav2 (if available)
ros2 launch nav2_bringup nav2_bringup_launch.py

# Start integration node
ros2 run autonomous_nav navigation_integration

# Send goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}}}}"
```

**Expected Output**: Robot navigates to goal while avoiding obstacles.

### Test 4: Complete System

```bash
# Launch all components
ros2 launch autonomous_nav autonomous_nav.launch.py
```

**Expected Output**: All components working together, robot navigating autonomously.

## Expected Outputs

### Object Detection

- Bounding boxes around detected objects
- Labels and confidence scores
- Annotated images published

### SLAM

- Occupancy grid map
- Robot pose estimates
- Map updating as robot moves

### Navigation

- Path planning to goals
- Obstacle avoidance
- Goal reaching

## Grading Rubric

### Object Detection (30 points)

- **YOLO Integration** (10 points): YOLO working correctly
- **Detection Publishing** (10 points): Detections published as ROS messages
- **Visualization** (10 points): Annotated images displayed

### SLAM (30 points)

- **Map Building** (15 points): Map being built
- **Localization** (15 points): Pose estimates accurate

### Navigation (30 points)

- **Nav2 Integration** (15 points): Nav2 working
- **Path Planning** (15 points): Paths planned correctly

### Integration (10 points)

- **System Integration** (5 points): All components working together
- **Performance** (5 points): Real-time operation

## Extensions

### Extension 1: Advanced SLAM

- Use RTAB-Map or ORB-SLAM3
- Loop closure detection
- 3D mapping

### Extension 2: Dynamic Obstacles

- Track moving objects
- Predict trajectories
- Dynamic replanning

### Extension 3: Semantic Navigation

- Navigate to specific objects
- Room-level navigation
- Task-based navigation

## Troubleshooting

### Issue: Objects not detected

**Solution**: Check camera topic, verify YOLO model loaded, check image format.

### Issue: Map not building

**Solution**: Verify camera data, check SLAM algorithm, verify pose updates.

### Issue: Navigation fails

**Solution**: Check Nav2 configuration, verify map available, check goal validity.

## Next Steps

- [Project 4: Voice-Controlled Robot Butler](project-04-vla.md) - VLA project
- [Module 4: Vision-Language-Action](../module-04-vla/vla-introduction) - Learn VLA

---

**Excellent work!** You've built a complete autonomous navigation system! 🚀

