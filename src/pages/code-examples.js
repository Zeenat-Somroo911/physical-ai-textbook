import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import clsx from 'clsx';
import styles from './code-examples.module.css';

export default function CodeExamples() {
  return (
    <Layout
      title="Code Examples"
      description="Complete, working code examples for Physical AI and Humanoid Robotics. All examples are tested and ready to use.">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className={styles.title}>Code Examples</h1>
            <p className={styles.subtitle}>
              Complete, working code examples for all modules. Copy, paste, and run!
            </p>

            <div className={styles.section}>
              <h2>Module 1: ROS 2 Fundamentals</h2>
              <div className={styles.exampleGrid}>
                <ExampleCard
                  title="Hello World Node"
                  description="Your first ROS 2 node in Python"
                  link="/docs/module-01-ros2/introduction#hello-world-node"
                  language="Python"
                />
                <ExampleCard
                  title="Publisher-Subscriber"
                  description="Complete pub-sub example with QoS"
                  link="/docs/module-01-ros2/nodes-topics#complete-example"
                  language="Python"
                />
                <ExampleCard
                  title="Service Client-Server"
                  description="Request-response pattern example"
                  link="/docs/module-01-ros2/services-actions#service-example"
                  language="Python"
                />
                <ExampleCard
                  title="Action Server-Client"
                  description="Long-running task example"
                  link="/docs/module-01-ros2/services-actions#action-example"
                  language="Python"
                />
                <ExampleCard
                  title="URDF Humanoid Robot"
                  description="Complete humanoid robot model"
                  link="/docs/module-01-ros2/urdf-basics#complete-humanoid-example"
                  language="XML"
                />
                <ExampleCard
                  title="Launch File"
                  description="Multi-node launch system"
                  link="/docs/module-01-ros2/launch-files#complete-example"
                  language="Python"
                />
              </div>
            </div>

            <div className={styles.section}>
              <h2>Module 2: Simulation & Gazebo</h2>
              <div className={styles.exampleGrid}>
                <ExampleCard
                  title="Gazebo World File"
                  description="Create custom simulation world"
                  link="/docs/module-02-simulation/gazebo-intro#world-files"
                  language="SDF"
                />
                <ExampleCard
                  title="URDF to SDF Conversion"
                  description="Convert robot models between formats"
                  link="/docs/module-02-simulation/urdf-sdf#conversion"
                  language="XML"
                />
                <ExampleCard
                  title="Physics Configuration"
                  description="Gravity, friction, and contact settings"
                  link="/docs/module-02-simulation/physics-simulation#physics-config"
                  language="SDF"
                />
                <ExampleCard
                  title="Camera Sensor"
                  description="RGB camera with noise model"
                  link="/docs/module-02-simulation/sensor-simulation#camera-example"
                  language="SDF"
                />
                <ExampleCard
                  title="LIDAR Sensor"
                  description="3D LIDAR simulation"
                  link="/docs/module-02-simulation/sensor-simulation#lidar-example"
                  language="SDF"
                />
                <ExampleCard
                  title="Complete Humanoid Lab"
                  description="Full humanoid robot in Gazebo"
                  link="/docs/module-02-simulation/practical-lab"
                  language="Multiple"
                />
              </div>
            </div>

            <div className={styles.section}>
              <h2>Module 3: AI-Powered Perception</h2>
              <div className={styles.exampleGrid}>
                <ExampleCard
                  title="PyBullet Robot"
                  description="Load and simulate robot in PyBullet"
                  link="/docs/module-03-isaac/pybullet-basics#complete-example"
                  language="Python"
                />
                <ExampleCard
                  title="OpenCV Camera Calibration"
                  description="Camera calibration with checkerboard"
                  link="/docs/module-03-isaac/computer-vision#camera-calibration"
                  language="Python"
                />
                <ExampleCard
                  title="YOLO Object Detection"
                  description="Real-time object detection"
                  link="/docs/module-03-isaac/computer-vision#object-detection"
                  language="Python"
                />
                <ExampleCard
                  title="ArUco Marker Detection"
                  description="Pose estimation with ArUco markers"
                  link="/docs/module-03-isaac/computer-vision#aruco-markers"
                  language="Python"
                />
                <ExampleCard
                  title="ORB-SLAM3 Integration"
                  description="SLAM with ROS 2"
                  link="/docs/module-03-isaac/slam-basics#complete-example"
                  language="C++/Python"
                />
                <ExampleCard
                  title="Nav2 Navigation"
                  description="Autonomous navigation stack"
                  link="/docs/module-03-isaac/navigation#complete-example"
                  language="Python/YAML"
                />
              </div>
            </div>

            <div className={styles.section}>
              <h2>Module 4: Vision-Language-Action</h2>
              <div className={styles.exampleGrid}>
                <ExampleCard
                  title="Web Speech API"
                  description="Browser-based voice recognition"
                  link="/docs/module-04-vla/voice-commands#web-speech-api"
                  language="JavaScript"
                />
                <ExampleCard
                  title="OpenAI Whisper"
                  description="Speech-to-text with Whisper API"
                  link="/docs/module-04-vla/voice-commands#whisper-api"
                  language="Python"
                />
                <ExampleCard
                  title="LLM Integration"
                  description="OpenAI API for robot commands"
                  link="/docs/module-04-vla/llm-integration#complete-example"
                  language="Python"
                />
                <ExampleCard
                  title="Action Planner"
                  description="Break commands into steps"
                  link="/docs/module-04-vla/action-planning#complete-planner"
                  language="Python"
                />
                <ExampleCard
                  title="Multimodal Pipeline"
                  description="Vision + Language + Action"
                  link="/docs/module-04-vla/multimodal#complete-pipeline"
                  language="Python"
                />
                <ExampleCard
                  title="Capstone Project"
                  description="Complete Autonomous Butler"
                  link="/docs/module-04-vla/capstone-project"
                  language="Multiple"
                />
              </div>
            </div>

            <div className={styles.section}>
              <h2>Projects</h2>
              <div className={styles.exampleGrid}>
                <ExampleCard
                  title="Multi-Robot Communication"
                  description="ROS 2 communication system"
                  link="/docs/projects/project-01-ros2-basics"
                  language="Python"
                />
                <ExampleCard
                  title="Build Your Own Humanoid"
                  description="Complete humanoid in Gazebo"
                  link="/docs/projects/project-02-simulation"
                  language="URDF/SDF"
                />
                <ExampleCard
                  title="Autonomous Navigation"
                  description="Object detection + SLAM + Nav2"
                  link="/docs/projects/project-03-perception"
                  language="Python"
                />
                <ExampleCard
                  title="Voice-Controlled Butler"
                  description="Complete VLA system"
                  link="/docs/projects/project-04-vla"
                  language="Multiple"
                />
              </div>
            </div>

            <div className={styles.ctaSection}>
              <h2>Need Help?</h2>
              <p>All code examples are embedded in their respective chapters with detailed explanations.</p>
              <div className={styles.ctaButtons}>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/intro">
                  Start Learning →
                </Link>
                <Link
                  className="button button--outline button--lg"
                  to="/docs/getting-started">
                  Setup Guide
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

function ExampleCard({title, description, link, language}) {
  return (
    <div className={styles.exampleCard}>
      <div className={styles.cardHeader}>
        <h3 className={styles.cardTitle}>{title}</h3>
        <span className={styles.languageBadge}>{language}</span>
      </div>
      <p className={styles.cardDescription}>{description}</p>
      <Link
        className={styles.cardLink}
        to={link}>
        View Example →
      </Link>
    </div>
  );
}

