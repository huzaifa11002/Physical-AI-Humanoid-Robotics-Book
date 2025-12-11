import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type Chapter = {
  title: string;
  link: string;
};

type ModuleItem = {
  title: string;
  link: string;
  chapters: Chapter[];
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: Robotic Nervous System',
    link: '/docs/robotic-nervous-system',
    chapters: [
      {title: 'Chapter 1: Intro to Physical AI', link: '/docs/robotic-nervous-system/introduction-physical-ai'},
      {title: 'Chapter 2: ROS 2 Architecture', link: '/docs/robotic-nervous-system/ros2-architecture'},
      {title: 'Chapter 3: Python Development', link: '/docs/robotic-nervous-system/ros2-python-development'},
      {title: 'Chapter 4: URDF Modeling', link: '/docs/robotic-nervous-system/urdf-robot-modeling'},
    ],
  },
  {
    title: 'Module 2: Digital Twin',
    link: '/docs/digital-twin',
    chapters: [
      {title: 'Chapter 5: Gazebo Fundamentals', link: '/docs/digital-twin/gazebo-fundamentals'},
      {title: 'Chapter 6: Physics Simulation', link: '/docs/digital-twin/physics-simulation'},
      {title: 'Chapter 7: Unity Integration', link: '/docs/digital-twin/unity-integration'},
    ],
  },
  {
    title: 'Module 3: AI Robot Brain',
    link: '/docs/ai-robot-brain',
    chapters: [
      {title: 'Chapter 8: NVIDIA Isaac Sim', link: '/docs/ai-robot-brain/nvidia-isaac-sim'},
      {title: 'Chapter 9: Perception', link: '/docs/ai-robot-brain/isaac-ros-perception'},
      {title: 'Chapter 10: Path Planning', link: '/docs/ai-robot-brain/nav2-path-planning'},
    ],
  },
  {
    title: 'Module 4: Vision Language Action',
    link: '/docs/vision-language-action',
    chapters: [
      {title: 'Chapter 11: Voice to Action', link: '/docs/vision-language-action/voice-to-action'},
      {title: 'Chapter 12: Cognitive Planning', link: '/docs/vision-language-action/llm-cognitive-planning'},
      {title: 'Chapter 13: Capstone Project', link: '/docs/vision-language-action/capstone-autonomous-humanoid'},
    ],
  },
];

function Module({title, link, chapters}: ModuleItem) {
  return (
    <div className={clsx(styles.feature)}>
      <Heading as="h3" className={styles.featureTitle}>
        {title}
      </Heading>
      <ul className={styles.chapterList}>
        {chapters.map((chapter, idx) => (
          <li key={idx} className={styles.chapterItem}>
            <Link className={styles.chapterLink} to={chapter.link}>
              {chapter.title}
            </Link>
          </li>
        ))}
      </ul>
      <Link className={styles.featureLink} to={link}>
        Learn More
      </Link>
    </div>
  );
}

export default function BookModules(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.features}>
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
