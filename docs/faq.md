---
sidebar_position: 4
---

# Frequently Asked Questions (FAQ)

Common questions and answers about the Physical AI & Humanoid Robotics course.

## General Questions

### Q1: Do I need a robot to take this course?

**A:** No! This course uses simulation extensively. You can complete the entire course using Gazebo and PyBullet simulations on your laptop. Real robot hardware is optional.

### Q2: What is the total cost of this course?

**A:** The course is essentially **free**! The only optional cost is $5 for OpenAI API credits in Module 4 (Vision-Language-Action). All other tools are completely free and open-source.

### Q3: How long does it take to complete the course?

**A:** 
- **Beginner**: 8-12 weeks (2-3 hours/day)
- **Intermediate**: 4-6 weeks (2-3 hours/day)
- **Advanced**: 2-3 weeks (intensive study)

### Q4: What programming languages do I need to know?

**A:** Python is the primary language. Basic Python knowledge (variables, functions, classes) is sufficient. We'll teach ROS 2 concepts as we go.

### Q5: Can I use Windows or Mac?

**A:** 
- **Windows**: Yes, using WSL2 (Windows Subsystem for Linux)
- **Mac**: Possible but requires additional setup (not officially supported)
- **Linux/Ubuntu**: Recommended and easiest

### Q6: Do I need a GPU?

**A:** No! All tools work on CPU. A GPU is optional and only speeds up some computer vision tasks, but not required.

## Installation and Setup

### Q7: ROS 2 installation fails. What should I do?

**A:** Common solutions:
1. Ensure you're on Ubuntu 22.04
2. Update system: `sudo apt update && sudo apt upgrade`
3. Check internet connection
4. Try installing components separately
5. See [Getting Started Guide](getting-started.md) for detailed steps

### Q8: "ros2: command not found" error

**A:** You need to source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```
Add this to your `~/.bashrc` to make it permanent.

### Q9: Gazebo won't start or crashes

**A:** Try these solutions:
1. Update graphics drivers: `sudo ubuntu-drivers autoinstall`
2. Use software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`
3. Check system requirements
4. Try Gazebo Classic instead of Gazebo Garden

### Q10: Python packages won't install

**A:** 
- Use `pip3 install --user package_name` (not sudo)
- Or create a virtual environment: `python3 -m venv venv && source venv/bin/activate`

## ROS 2 Questions

### Q11: What's the difference between ROS 1 and ROS 2?

**A:** ROS 2 is the modern version with:
- Better real-time support
- Improved security
- Cross-platform support
- Better DDS integration
- Active development

**This course uses ROS 2.**

### Q12: How do I create a ROS 2 package?

**A:**
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_package
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Q13: My node isn't publishing messages. Why?

**A:** Common issues:
1. Check topic name spelling: `ros2 topic list`
2. Verify node is running: `ros2 node list`
3. Check message type matches: `ros2 topic info /topic_name`
4. Ensure publisher is actually publishing (check code)

### Q14: How do I debug ROS 2 nodes?

**A:** 
- Use `ros2 node info /node_name` to see connections
- Check logs: `ros2 run package_name node_name`
- Use `rqt_graph` to visualize node connections
- Add `self.get_logger().info()` statements in code

### Q15: What's the difference between topics, services, and actions?

**A:**
- **Topics**: One-way communication (publisher → subscriber)
- **Services**: Request-response (client → server)
- **Actions**: Long-running tasks with feedback (client → server with progress)

See [Module 1, Chapter 3](module-01-ros2/03-services-actions.md) for details.

## Simulation Questions

### Q16: Gazebo is very slow. How can I speed it up?

**A:**
1. Reduce physics update rate in world file
2. Simplify robot models (fewer links/joints)
3. Disable unnecessary sensors
4. Use headless mode: `gazebo --headless`
5. Close other applications

### Q17: How do I convert URDF to SDF?

**A:**
```bash
gz sdf -p my_robot.urdf > my_robot.sdf
```
Or use online converters or Gazebo's built-in conversion.

### Q18: My robot model doesn't appear in Gazebo

**A:** Check:
1. URDF/SDF syntax is correct
2. File paths in URDF are absolute or correct relative paths
3. Model is in Gazebo's model path
4. Check Gazebo console for errors

### Q19: Can I use Unity instead of Gazebo?

**A:** Yes! See [Module 2, Chapter 5](module-02-simulation/05-unity-integration.md) for Unity integration with ROS 2.

## Computer Vision Questions

### Q20: OpenCV installation fails

**A:**
```bash
# Try pip install
pip3 install --user opencv-python

# Or install from apt
sudo apt install -y python3-opencv
```

### Q21: YOLO detection is slow

**A:**
1. Use smaller model: `yolov8n.pt` (nano) instead of `yolov8x.pt`
2. Reduce image resolution
3. Process every Nth frame
4. Use GPU if available

### Q22: Camera not detected in ROS 2

**A:**
1. Check camera is connected: `ls /dev/video*`
2. Install camera drivers if needed
3. Use `v4l2-ctl --list-devices` to list cameras
4. Check camera permissions

## VLA and AI Questions

### Q23: OpenAI API costs too much

**A:** 
- Use GPT-3.5-turbo (cheaper than GPT-4)
- Cache common queries
- Use function calling to reduce tokens
- $5 credit is enough for the entire course

### Q24: Voice recognition doesn't work

**A:**
1. Check microphone permissions
2. Test microphone: `arecord -d 5 test.wav && aplay test.wav`
3. Reduce background noise
4. Speak clearly and slowly
5. Try different recognition engines

### Q25: LLM generates invalid plans

**A:**
1. Use structured prompts with examples
2. Validate plan format before execution
3. Add error handling
4. Use function calling for structured output
5. Fine-tune prompts based on failures

## Hardware Questions

### Q26: What robot should I buy?

**A:** For this course, **no robot needed**! But if you want hardware:
- **Beginner**: TurtleBot 3, ROSbot
- **Intermediate**: Fetch, TIAGo
- **Advanced**: Custom build, humanoid platforms

### Q27: Can I use a Raspberry Pi?

**A:** Yes! Raspberry Pi 4 (4GB+) can run ROS 2 Humble. However:
- Simulation will be slow
- Use lightweight models
- Consider remote development (code on PC, run on Pi)

### Q28: What sensors do I need?

**A:** For simulation: None! For real robots:
- **Camera**: USB webcam (cheap)
- **LIDAR**: Optional, expensive
- **IMU**: Often built into robot
- **Encoders**: Usually built into motors

## Career Questions

### Q29: What jobs can I get with these skills?

**A:**
- Robotics Software Engineer
- Autonomous Systems Engineer
- Computer Vision Engineer
- ROS Developer
- Research Engineer
- Robotics Consultant

### Q30: How do I build a portfolio?

**A:**
1. Complete all course projects
2. Build your own robot projects
3. Contribute to open-source ROS packages
4. Document your work (GitHub, blog)
5. Share demos on YouTube/LinkedIn

### Q31: Should I get a degree in robotics?

**A:** Not required! Many successful robotics engineers are self-taught. However:
- **Degree**: Provides structure, credentials, networking
- **Self-taught**: Faster, cheaper, more flexible
- **Best**: Combine both (degree + self-learning)

### Q32: How do I stay updated with robotics?

**A:**
1. Follow ROS Discourse
2. Read robotics papers (arXiv)
3. Join robotics communities
4. Attend conferences (virtual or in-person)
5. Contribute to open-source projects

## Troubleshooting

### Q33: Everything is broken! Help!

**A:** Don't panic! Try:
1. **Restart**: Reboot your computer
2. **Clean build**: `rm -rf build install log && colcon build`
3. **Reinstall**: Start fresh with clean Ubuntu install
4. **Ask for help**: Post on forums with error messages
5. **Check logs**: Look at terminal output for clues

### Q34: I'm stuck on a concept. What should I do?

**A:**
1. Re-read the chapter
2. Try the code examples
3. Search online (Stack Overflow, ROS Discourse)
4. Ask in community forums
5. Take a break and come back fresh

### Q35: The course is too fast/slow for me

**A:**
- **Too fast**: Take your time, repeat chapters, do all exercises
- **Too slow**: Skip ahead, focus on projects, explore advanced topics
- **Customize**: Learn at your own pace!

## Getting Help

### Q36: Where can I get help?

**A:**
1. **FAQ**: Check this page first
2. **Getting Started Guide**: [getting-started.md](getting-started.md)
3. **Glossary**: [glossary.md](glossary.md) for term definitions
4. **Community Forums**: ROS Discourse, Reddit
5. **GitHub Issues**: [Report bugs or ask questions](https://github.com/Zeenat-Somroo911/physical-ai-textbook/issues)

### Q37: How do I report a bug in the textbook?

**A:**
1. Check if it's already reported
2. Create a GitHub issue with:
   - Clear description
   - Steps to reproduce
   - Expected vs actual behavior
   - System information

### Q38: Can I contribute to the textbook?

**A:** Yes! We welcome contributions:
1. Fix typos and errors
2. Improve explanations
3. Add code examples
4. Write new chapters
5. Translate to other languages

See our [GitHub repository](https://github.com/Zeenat-Somroo911/physical-ai-textbook) for contribution guidelines.

---

**Still have questions?** Check the [Resources](resources.md) page or ask in our community forums!

