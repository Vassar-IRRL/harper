import type {CSSProperties, ReactNode} from 'react';
import {useLayoutEffect, useState} from 'react';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './page.module.css';

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

type PersonLink = {
  label: string;
  href: string;
};

type Person = {
  name: string;
  role: string;
  image?: string;
  links?: PersonLink[];
};

const currentMembers: Person[] = [
  {
    name: 'Kenneth R. Livingston',
    role: 'Professor of Cognitive Science',
    links: [{label: 'Email', href: 'mailto:livingst@vassar.edu'}],
  },
  {
    name: 'Klaus Misko',
    role: 'Student researcher',
    links: [{label: 'Email', href: 'mailto:nmisko@vassar.edu'}],
  },
  {
    name: 'Samip Paudel',
    role: 'Student researcher',
    links: [
      {label: 'Email', href: 'mailto:spaudel@vassar.edu'},
      {label: 'Github', href: 'https://www.github.com/smpdl'},
      {label: 'Website', href: 'https://www.paudelsamip.com.np'},
    ],
  },

  {
    name: 'Maria Petroiesc',
    role: 'Student researcher',
    links: [{label: 'Email', href: 'mailto:mpetroiesc@vassar.edu'}],
  },
];

/** const pastMembers: Person[] = [
  {
    name: 'Past member 1',
    role: 'Student researcher',
  },
  {
    name: 'Past member 2',
    role: 'Student researcher',
  },
  {
    name: 'Past member 3',
    role: 'Student researcher',
  },
];
**/

function PersonPhoto({person}: {person: Person}): ReactNode {
  if (person.image) {
    return (
      <img
        className={styles.personPhoto}
        src={person.image}
        alt={person.name}
      />
    );
  }

  return <div className={styles.personPhotoPlaceholder} aria-hidden="true" />;
}

function PeopleSection({
  title,
  members,
}: {
  title: string;
  members: Person[];
}): ReactNode {
  if (members.length === 0) {
    return null;
  }

  return (
    <section className={styles.peopleSection}>
      <Heading as="h2" className={styles.peopleSectionTitle}>
        {title}
      </Heading>
      <div className={styles.peopleGrid}>
        {members.map((person) => (
          <article key={person.name} className={styles.personCard}>
            <PersonPhoto person={person} />
            <div className={styles.personBody}>
              <Heading as="h3" className={styles.personName}>
                {person.name}
              </Heading>
              <p className={styles.personRole}>{person.role}</p>
              {person.links && person.links.length > 0 ? (
                <ul className={styles.personLinks}>
                  {person.links.map((link) => (
                    <li key={`${link.label}-${link.href}`}>
                      <a
                        href={link.href}
                        {...(link.href.startsWith('http')
                          ? {target: '_blank', rel: 'noopener noreferrer'}
                          : {})}>
                        {link.label}
                      </a>
                    </li>
                  ))}
                </ul>
              ) : null}
            </div>
          </article>
        ))}
      </div>
    </section>
  );
}

export default function PeoplePage(): ReactNode {
  const homeInset = useHomeNavInset();
  const contentAlignStyle: CSSProperties | undefined =
    homeInset !== undefined
      ? {paddingLeft: `${homeInset}px`, paddingRight: `${homeInset}px`}
      : undefined;

  return (
    <Layout
      title="People"
      description="People behind the project!">
      <main className={styles.page}>
        <div className={styles.pageInner} style={contentAlignStyle}>
          <Heading as="h1" className={styles.pageTitle}>
            People
          </Heading>
          <p className={styles.pageIntro}>
            Multiple people have been part of this project in various capacities to help it be where it is right now. Feel free to reach out to the current members of the project for any help or if you want to learn more.
          </p>
          <PeopleSection title="Current members" members={currentMembers} />
          <section className={styles.supportNote}>
            <Heading as="h2" className={styles.supportTitle}>
              Support
            </Heading>
            <p className={styles.supportText}>
              The Undergraduate Research Summer Institute (URSI) at Vassar
              College supports this project. URSI is a summer program for STEM Research at Vassar. The Department of Cognitive Science at Vassar College also supports this project! 
            </p>
          </section>
        </div>
      </main>
    </Layout>
  );
}
