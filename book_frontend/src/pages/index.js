import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="text-center">
          <Heading as="h1" className="hero__title text-4xl md:text-5xl font-bold mb-4">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle text-xl mb-8 max-w-2xl mx-auto">
            {siteConfig.tagline}
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg bg-blue-600 hover:bg-blue-700 text-white rounded-lg px-8 py-3 font-semibold transition-all duration-300 transform hover:scale-105"
              to="/docs/intro">
              Get Started - 5min ⏱️
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main className="py-12">
        <div className="container mx-auto px-4">
          <HomepageFeatures />
        </div>
      </main>
    </Layout>
  );
}
