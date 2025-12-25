import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: ROS 2 Nervous System',
    description: [
      'Learn the fundamentals of ROS 2 architecture',
      'Understand nodes, topics, and services communication',
      'Master the ROS 2 command-line tools'
    ],
  },
  {
    title: 'Module 2: Digital Twin',
    description: [
      'Create realistic 3D robot models and environments',
      'Simulate robot behavior in virtual worlds',
      'Connect simulation with real-world data'
    ],
  },
  {
    title: 'Module 3: AI Robot Brain',
    description: [
      'Implement perception systems for robot awareness',
      'Develop decision-making algorithms',
      'Apply machine learning for robot behavior'
    ],
  },
  {
    title: 'Module 4: VLA Robotics',
    description: [
      'Explore vision-language-action models',
      'Implement multimodal AI for robotics',
      'Connect language understanding with robot actions'
    ],
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--3')}>
      <div className="text--center padding-horiz--md">
        <div style={{background: 'white', boxShadow: '0 4px 10px rgba(0,0,0,0.2)', borderRadius: '0.5rem', padding: '1.25rem', height: '100%', display: 'flex', flexDirection: 'column', border: 'none'}} className="h-full flex flex-col">
          <Heading as="h3" className="text-lg font-semibold mb-3">{title}</Heading>
          <ul className="flex-grow text-left text-sm text-gray-600 space-y-1">
            {description.map((item, index) => (
              <li key={index} className="flex items-start">
                <span className="mr-2">•</span>
                <span>{item}</span>
              </li>
            ))}
          </ul>
        </div>
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
