---
sidebar_position: 5
---

# Glossary

Alphabetical glossary of terms used in Physical AI & Humanoid Robotics.

## A

**Action**: A ROS 2 communication mechanism for long-running tasks with feedback. Actions consist of a goal, feedback, and result.

**Actuator**: A device that converts control signals into physical motion (e.g., motors, servos).

**ArUco Marker**: A binary square marker used for camera pose estimation and robot localization.

**Autonomous**: A system capable of operating independently without human intervention.

## B

**Bag File**: A ROS file format for recording and playing back topic data (`.bag` files).

**Base Link**: The root link of a robot's kinematic chain in URDF.

**Bounding Box**: A rectangular box that encloses a detected object in computer vision.

**Bullet Physics**: A physics engine used in PyBullet for robot simulation.

## C

**Callback**: A function that is called when a specific event occurs (e.g., receiving a message).

**Camera Calibration**: The process of determining a camera's intrinsic and extrinsic parameters.

**Collision Geometry**: The simplified shape used for collision detection in physics simulation.

**Computer Vision**: The field of AI that enables computers to interpret and understand visual information.

**Costmap**: A 2D grid map used in navigation that assigns costs to different areas based on obstacles.

**CV Bridge**: A ROS package that converts between ROS Image messages and OpenCV image formats.

## D

**DDS (Data Distribution Service)**: The middleware protocol used by ROS 2 for communication.

**Dead Reckoning**: Estimating position using previous position, velocity, and time (without external references).

**Depth Estimation**: Determining the distance to objects in a scene using stereo vision or depth cameras.

**Docker**: A containerization platform for packaging and running applications.

**Doxygen**: A documentation generation tool for code.

## E

**Encoder**: A sensor that measures the position or velocity of a motor shaft.

**End Effector**: The tool or gripper at the end of a robot arm.

**Euler Angles**: A method of representing 3D rotations using three angles (roll, pitch, yaw).

## F

**Feedback**: Information sent back to indicate progress of a long-running task.

**Frame**: A coordinate system in ROS (e.g., `base_link`, `map`, `camera_frame`).

**Frame Transform**: The relationship between two coordinate frames (position and orientation).

**Function Calling**: A feature of LLMs that allows structured output in JSON format.

## G

**Gazebo**: A 3D physics simulation environment for robots.

**Gripper**: A robot end effector designed to grasp and manipulate objects.

**Ground Truth**: The actual, correct value used to evaluate model performance.

## H

**Humanoid Robot**: A robot designed to resemble the human body in form and function.

**Humble Hawksbill**: The codename for ROS 2 Humble distribution (Ubuntu 22.04).

## I

**IMU (Inertial Measurement Unit)**: A sensor that measures acceleration and angular velocity.

**Inertia**: The resistance of an object to changes in its motion.

**Intrinsic Parameters**: Camera parameters that describe the internal geometry (focal length, principal point).

## J

**Joint**: A connection between two links in a robot that allows relative motion.

**Joint State**: The position, velocity, and effort of all joints in a robot.

**JSON (JavaScript Object Notation)**: A lightweight data format used for configuration and data exchange.

## K

**Kinematics**: The study of motion without considering forces (forward and inverse kinematics).

**Kinetic Chain**: A series of connected links and joints in a robot.

## L

**Launch File**: A ROS file that starts multiple nodes and configures the system.

**LIDAR (Light Detection and Ranging)**: A sensor that uses laser pulses to measure distances.

**Link**: A rigid body component in a robot model (URDF).

**LLM (Large Language Model)**: An AI model trained on vast amounts of text data (e.g., GPT-3.5, GPT-4).

**Localization**: Determining a robot's position and orientation in an environment.

## M

**Map**: A representation of the environment, typically a 2D occupancy grid.

**Marker**: A visual fiducial used for pose estimation (e.g., ArUco markers).

**Mermaid**: A diagramming and charting tool used in markdown documentation.

**Message**: A data structure used for communication between ROS nodes.

**Middleware**: Software that enables communication between distributed components (DDS in ROS 2).

**Multimodal**: Systems that process multiple types of input (e.g., vision, language, audio).

## N

**Nav2**: The ROS 2 navigation stack for autonomous mobile robot navigation.

**Node**: A process in ROS that performs computation and communicates with other nodes.

**Noise**: Random variations in sensor measurements.

## O

**Object Detection**: Identifying and locating objects in images or video.

**Occupancy Grid**: A 2D grid map where each cell represents whether that space is occupied or free.

**Odometry**: Estimation of a robot's position based on wheel encoders or other motion sensors.

**OpenCV**: An open-source computer vision library.

**ORB-SLAM3**: An open-source SLAM system for visual-inertial and RGB-D cameras.

**Orientation**: The rotation of an object in 3D space (quaternion, Euler angles).

## P

**Parameter**: A configuration value stored in a ROS node.

**Path Planning**: Computing a collision-free path from start to goal.

**Perception**: The process of interpreting sensor data to understand the environment.

**Physics Engine**: Software that simulates physical interactions (gravity, collisions, etc.).

**Pose**: The position and orientation of an object in space.

**Pose Estimation**: Determining the 3D position and orientation of objects.

**Publisher**: A ROS node that sends messages on a topic.

**PyBullet**: A Python physics simulation library.

**Python**: The primary programming language used in this course.

## Q

**QoS (Quality of Service)**: ROS 2 settings that control message delivery reliability and ordering.

**Quaternion**: A mathematical representation of 3D rotation using four numbers (w, x, y, z).

## R

**RAG (Retrieval-Augmented Generation)**: A technique that enhances LLM responses with retrieved information.

**rclpy**: The ROS 2 Python client library.

**Real-time**: Systems that must respond within strict time constraints.

**RGB-D Camera**: A camera that captures both color (RGB) and depth information.

**Robot**: A programmable machine capable of carrying out complex actions automatically.

**ROS (Robot Operating System)**: A framework for writing robot software.

**ROS 2**: The second generation of ROS with improved real-time support and security.

**RViz2**: A 3D visualization tool for ROS 2.

## S

**SDF (Simulation Description Format)**: An XML format for describing robots and environments in Gazebo.

**Sensor**: A device that measures physical properties (camera, LIDAR, IMU, etc.).

**Service**: A ROS 2 communication mechanism for request-response interactions.

**SLAM (Simultaneous Localization and Mapping)**: Building a map while simultaneously localizing within it.

**Subscriber**: A ROS node that receives messages from a topic.

## T

**TF (Transform)**: The ROS library for managing coordinate frame transformations.

**Topic**: A named communication channel in ROS for message passing.

**Trajectory**: A sequence of poses that a robot follows.

**Twist**: A ROS message type representing linear and angular velocity.

## U

**URDF (Unified Robot Description Format)**: An XML format for describing robot models.

**Ubuntu**: A Linux distribution, recommended for ROS 2 development.

## V

**VLA (Vision-Language-Action)**: Systems that combine visual perception, natural language understanding, and action execution.

**Visual Geometry**: The detailed 3D model used for rendering in simulation.

**Visual Odometry**: Estimating robot motion using camera images.

**Voice Recognition**: Converting spoken words into text.

## W

**Waypoint**: A specific location that a robot should navigate to.

**World File**: A Gazebo file that describes the simulation environment.

**WSL2 (Windows Subsystem for Linux)**: A compatibility layer for running Linux on Windows.

## X

**XML (eXtensible Markup Language)**: A markup language used for URDF and SDF files.

## Y

**YAML (YAML Ain't Markup Language)**: A human-readable data format used for configuration files.

**YOLO (You Only Look Once)**: A real-time object detection algorithm.

## Z

**Zero Configuration**: A feature that allows ROS 2 nodes to discover each other automatically.

---

**Need more definitions?** Check the [Resources](resources.md) page for additional learning materials!

