import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description="Open-source hardware and software for the HARPER humanoid robotics platform.">
      <header className={clsx('hero', styles.hero)}>
        <div className="container">
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroTagline}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            A low-cost, 3D-printed humanoid robot platform for researchers in
            behavioral and cognitive science and for hobbyist use. Developed at
            the Interdisciplinary Robotics Research Lab (IRRL) and the Cognitive
            Science Department at Vassar College.
          </p>
          <div className={styles.buttons}>
            <Link className="button button--primary button--lg" to="/docs/intro">
              Read the docs
            </Link>
            <Link
              className="button button--secondary button--lg"
              href="https://github.com/Vassar-IRRL/harper">
              View on GitHub
            </Link>
          </div>
        </div>
      </header>
    </Layout>
  );
}
