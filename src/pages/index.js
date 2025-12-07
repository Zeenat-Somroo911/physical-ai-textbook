import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

import styles from './index.module.css';

function HeroSection() {
  const { siteConfig } = useDocusaurusContext();
  const logoUrl = useBaseUrl('img/logo.png');
  return (
    <section className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.gradientOrb1}></div>
        <div className={styles.gradientOrb2}></div>
        <div className={styles.gradientOrb3}></div>
      </div>
      <div className={styles.heroContent}>
        <div className="container">
          <div className={styles.heroLogo}>
            <img
              src={logoUrl}
              alt="Physical AI Logo"
              className={styles.logoImage}
              onError={(e) => {
                // Hide image if it doesn't exist
                e.target.style.display = 'none';
              }}
            />
          </div>
          <h1 className={styles.heroTitle}>
            Master <span className={styles.gradientText}>Physical AI</span> &<br />
            Humanoid Robotics
          </h1>
          <p className={styles.heroSubtitle}>
            Build intelligent robots that understand, perceive, and act in the real world.
            Learn ROS 2, simulation, perception, and VLA systems—all for free.
          </p>
          <div className={styles.heroButtons}>
            <Link
              className={clsx('button button--primary button--lg', styles.ctaButton)}
              to="/docs/intro">
              Start Learning →
            </Link>
            <button
              className={clsx('button button--outline button--lg', styles.ctaButtonSecondary)}
              onClick={() => {
                const modulesSection = document.querySelector('.' + styles.modules);
                if (modulesSection) {
                  const yOffset = -80;
                  const y = modulesSection.getBoundingClientRect().top + window.pageYOffset + yOffset;
                  window.scrollTo({ top: y, behavior: 'smooth' });
                }
              }}
            >
              View Content
            </button>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>4</div>
              <div className={styles.statLabel}>Modules</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>25+</div>
              <div className={styles.statLabel}>Chapters</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>$0-5</div>
              <div className={styles.statLabel}>Total Cost</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>100%</div>
              <div className={styles.statLabel}>Free Tools</div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🤖',
      title: 'ROS 2 Fundamentals',
      description: 'Master the Robot Operating System 2 from the ground up. Learn nodes, topics, services, and actions.',
      link: '/docs/module-01-ros2/introduction',
    },
    {
      icon: '🌐',
      title: 'Simulation & Gazebo',
      description: 'Build and simulate robots in virtual environments. Create realistic physics simulations and test before deployment.',
      link: '/docs/module-02-simulation/gazebo-intro',
    },
    {
      icon: '👁️',
      title: 'AI-Powered Perception',
      description: 'Implement computer vision, SLAM, and navigation using free, open-source tools. No expensive hardware needed.',
      link: '/docs/module-03-isaac/free-alternatives',
    },
    {
      icon: '🧠',
      title: 'Vision-Language-Action',
      description: 'Build robots that understand natural language. Integrate voice commands, LLMs, and multimodal systems.',
      link: '/docs/module-04-vla/vla-introduction',
    },
    {
      icon: '💻',
      title: 'Hands-On Projects',
      description: 'Complete 5 comprehensive projects from multi-robot systems to autonomous navigation and VLA butlers.',
      link: '/docs/projects/overview',
    },
    {
      icon: '🚀',
      title: 'Production Ready',
      description: 'Learn industry best practices. Build real-world robotics systems that are robust, scalable, and maintainable.',
      link: '/docs/intro',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Why Choose This Course?</h2>
          <p className={styles.sectionSubtitle}>
            Everything you need to become a robotics engineer, all in one place
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div key={idx} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
              <Link to={feature.link} className={styles.featureLink}>
                Learn More →
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulesSection() {
  const modules = [
    {
      number: '01',
      title: 'ROS 2 Fundamentals',
      description: 'Master the Robot Operating System 2. Learn nodes, topics, services, actions, URDF, and launch files.',
      chapters: 6,
      duration: '2-3 weeks',
      link: '/docs/module-01-ros2/introduction',
      color: 'blue',
    },
    {
      number: '02',
      title: 'Simulation & Gazebo',
      description: 'Build robots in virtual environments. Learn Gazebo, URDF/SDF, physics simulation, and sensor models.',
      chapters: 6,
      duration: '2-3 weeks',
      link: '/docs/module-02-simulation/gazebo-intro',
      color: 'green',
    },
    {
      number: '03',
      title: 'AI-Powered Perception',
      description: 'Free alternatives to expensive tools. Learn PyBullet, OpenCV, ORB-SLAM3, and Nav2—all open-source.',
      chapters: 6,
      duration: '2-3 weeks',
      link: '/docs/module-03-isaac/free-alternatives',
      color: 'purple',
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      description: 'Build robots that understand natural language. Voice commands, LLM integration, and complete VLA systems.',
      chapters: 6,
      duration: '2-3 weeks',
      link: '/docs/module-04-vla/vla-introduction',
      color: 'orange',
    },
  ];

  return (
    <section className={styles.modules}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Course Modules</h2>
          <p className={styles.sectionSubtitle}>
            A comprehensive curriculum covering everything from basics to advanced VLA systems
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <div key={idx} className={clsx(styles.moduleCard, styles[`moduleCard${module.color}`])}>
              <div className={styles.moduleNumber}>{module.number}</div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <p className={styles.moduleDescription}>{module.description}</p>
              <div className={styles.moduleMeta}>
                <span className={styles.moduleMetaItem}>{module.chapters} Chapters</span>
                <span className={styles.moduleMetaItem}>{module.duration}</span>
              </div>
              <Link to={module.link} className={styles.moduleLink}>
                Start Module {module.number} →
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsSection() {
  const stats = [
    { number: '4', label: 'Comprehensive Modules', icon: '📚' },
    { number: '25+', label: 'Detailed Chapters', icon: '📖' },
    { number: '5', label: 'Hands-On Projects', icon: '🛠️' },
    { number: '$0-5', label: 'Total Cost', icon: '💰' },
    { number: '100%', label: 'Free & Open Source', icon: '🆓' },
    { number: '24/7', label: 'Self-Paced Learning', icon: '⏰' },
  ];

  return (
    <section className={styles.stats}>
      <div className="container">
        <div className={styles.statsGrid}>
          {stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statIcon}>{stat.icon}</div>
              <div className={styles.statNumber}>{stat.number}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className={styles.ctaBackground}>
        <div className={styles.ctaGradient}></div>
      </div>
      <div className="container">
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>Ready to Build Amazing Robots?</h2>
          <p className={styles.ctaSubtitle}>
            Join thousands of learners building the future of robotics. Start your journey today—it's completely free!
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button button--primary button--lg', styles.ctaButton)}
              to="/docs/intro">
              Start Learning Now →
            </Link>
            <button
              className={clsx('button button--outline button--lg', styles.ctaButtonOutline)}
              onClick={() => {
                const modulesSection = document.querySelector('.' + styles.modules);
                if (modulesSection) {
                  const yOffset = -80;
                  const y = modulesSection.getBoundingClientRect().top + window.pageYOffset + yOffset;
                  window.scrollTo({ top: y, behavior: 'smooth' });
                }
              }}
            >
              View Content
            </button>
          </div>
          <div className={styles.ctaFeatures}>
            <div className={styles.ctaFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>No credit card required</span>
            </div>
            <div className={styles.ctaFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>100% free tools</span>
            </div>
            <div className={styles.ctaFeature}>
              <span className={styles.checkmark}>✓</span>
              <span>Self-paced learning</span>
            </div>
          </div>
        </div>
      </div>
    </section >
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Embodied Intelligence - A comprehensive textbook on Physical AI and Humanoid Robotics. Learn ROS 2, simulation, perception, and VLA systems for free.">
      <main className={styles.homepage}>
        <HeroSection />
        <FeaturesSection />
        <ModulesSection />
        <StatsSection />
        <CTASection />
      </main>
    </Layout>
  );
}
