import type {CSSProperties, ReactNode} from 'react';
import {useLayoutEffect, useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function useHomeNavInset(): number | undefined {
  const [inset, setInset] = useState<number | undefined>(undefined);

  useLayoutEffect(() => {
    const sync = () => {
      const homeLink = Array.from(
        document.querySelectorAll<HTMLElement>('.navbar__link'),
      ).find((el) => el.textContent?.trim() === 'Home');
      if (!homeLink) {
        return;
      }
      const paddingLeft = parseFloat(getComputedStyle(homeLink).paddingLeft) || 0;
      setInset(homeLink.getBoundingClientRect().left + paddingLeft);
    };

    sync();
    window.addEventListener('resize', sync);
    return () => window.removeEventListener('resize', sync);
  }, []);

  return inset;
}

function Section({
  id,
  title,
  children,
  contentStyle,
}: {
  id: string;
  title: string;
  children: ReactNode;
  contentStyle?: CSSProperties;
}): ReactNode {
  return (
    <section id={id} className={styles.section}>
      <div className={styles.sectionInner} style={contentStyle}>
        <Heading as="h2" className={styles.sectionTitle}>
          {title}
        </Heading>
        {children}
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const homeInset = useHomeNavInset();
  const heroAlignStyle: CSSProperties | undefined =
    homeInset !== undefined ? {paddingLeft: `${homeInset}px`} : undefined;
  const contentAlignStyle: CSSProperties | undefined =
    homeInset !== undefined
      ? {paddingLeft: `${homeInset}px`, paddingRight: `${homeInset}px`}
      : undefined;

  return (
    <Layout
      title="Home"
      description="Open-source hardware and software for the HARPER humanoid robotics platform.">
      <header className={clsx('hero', styles.hero)}>
        <div className={styles.heroInner}>
          <div className={styles.heroCopy} style={heroAlignStyle}>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroTagline}>{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              HARPER is a low-cost, 3D-printed humanoid robot platform for researchers in
              behavioral and cognitive science and for hobbyist use. Built at the Interdisciplinary Robotics Research Lab (IRRL) at Vassar College,
              HARPER's software and hardware are open-source and free to use. 
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/">
                Get started
              </Link>
              <Link
                className="button button--primary button--lg"
                href="https://github.com/Vassar-IRRL/harper">
                View on GitHub
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <img
              src={useBaseUrl('/img/harper-visual.png')}
              alt="HARPER dual-arm humanoid robot"
              className={styles.heroImage}
            />
          </div>
        </div>
      </header>

      <main>
        <Section
          id="about"
          title="About the project"
          contentStyle={contentAlignStyle}>
          <p className={styles.prose}>
            Robotics has always had a barrier of entry for many researchers and hobbyists due to the high cost of building and buying robotic platforms. Someone who wants to run an experiment or learn more about robotics needs to either build their own robot or buy a commercial robot. This is often a huge investment, which most labs and hobbyists cannot afford. HARPER was built to address this problem by providing a low-cost, open-source platform for people to use and build on.
          </p>
          <p className={styles.prose}>
            Given that 3D printing is becoming more and more accessible, we believe that it is a great way to get started with robotics. This is why we decided to build HARPER as a 3D-printed humanoid robot platform. All major parts of HARPER are designed to be printed on a standard 3D printer. It is actuated entirely by Dynamixel servos, which are also very accessible. 
          </p>
          <p className={styles.prose}>
            To get people started easily, we have also created a comprehensive set of guides and documentation. The software stack is built on top of ROS2. 
          </p>
        </Section>

        <Section
          id="note"
          title="Note"
          contentStyle={contentAlignStyle}>
          <p className={styles.prose}>
            HARPER is still under development and is not yet mature enough for end to end use. This is a beta build and is still being heavily tested and developed. If you have any questions or feedback, please don't hesitate to contact us (<a href="mailto:spaudel@vassar.edu">email</a>). All community discussions are welcome on our github repository (<a href="https://github.com/Vassar-IRRL/harper/discussions">discussions</a>).
          </p>
        </Section>

        <Section
          id="resources"
          title="Resources"
          contentStyle={contentAlignStyle}>
          <div className={styles.projectGrid}>
            <ul className={styles.projectLinks}>
            <li>
                <Link to="/docs/">
                  Project Documentation
                </Link>
              </li>
              <li>
                <Link href="https://github.com/Vassar-IRRL/harper">
                  GitHub repository
                </Link>
              </li>
              <li>
                <Link href="https://github.com/Vassar-IRRL/harper/blob/main/CONTRIBUTING">
                  Contributing
                </Link>
              </li>
              <li>
                <Link href="https://github.com/Vassar-IRRL/harper/blob/main/LICENSE">
                  Licenses
                </Link>
              </li>
            </ul>
          </div>
        </Section>
      </main>
    </Layout>
  );
}

