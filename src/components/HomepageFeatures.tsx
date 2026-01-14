import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

type FeatureItem = {
  title: string;
  description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Concepts',
    description: (
      <>
        Learn how artificial intelligence integrates with physical systems to create intelligent robots.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    description: (
      <>
        Explore the unique challenges and solutions in humanoid robotics design and control.
      </>
    ),
  },
  {
    title: 'Hands-on Exercises',
    description: (
      <>
        Apply theoretical concepts through practical exercises using ROS 2, Gazebo, Unity, and NVIDIA Isaac.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
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