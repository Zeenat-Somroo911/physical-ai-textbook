import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: ROS2 Fundamentals',
    emoji: '🤖',
    description: (
      <>
        Master the Robot Operating System 2 (ROS2) from the ground up. Learn about nodes, topics, services, actions, and best practices for building robust robotic systems.
      </>
    ),
    link: '/docs/module-01-ros2/intro',
  },
  {
    title: 'Module 2: Simulation & Gazebo',
    emoji: '🌐',
    description: (
      <>
        Dive into robot simulation using Gazebo. Create robot models, configure sensors and actuators, and build realistic simulation environments for testing and development.
      </>
    ),
    link: '/docs/module-02-simulation/intro',
  },
  {
    title: 'Module 3: NVIDIA Isaac Sim',
    emoji: '🎮',
    description: (
      <>
        Explore NVIDIA Isaac Sim for high-fidelity robot simulation. Learn scene composition, physics engines, and ROS bridge integration for advanced robotics workflows.
      </>
    ),
    link: '/docs/module-03-isaac/intro',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    emoji: '🧠',
    description: (
      <>
        Understand Vision-Language-Action models that enable robots to understand natural language, perceive their environment, and execute complex tasks autonomously.
      </>
    ),
    link: '/docs/module-04-vla/intro',
  },
];

function Feature({emoji, title, description, link}) {
  return (
    <div className={clsx('col col--6')}>
      <div className="text--center">
        <div className={styles.featureIcon}>{emoji}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
        <a href={link} className="button button--primary button--outline">
          Learn More →
        </a>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

