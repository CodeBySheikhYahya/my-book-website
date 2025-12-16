import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A Complete Guide to Building Intelligent Robots">
      <main style={{padding: '2rem', textAlign: 'center'}}>
        <h1>{siteConfig.title}</h1>
        <p style={{fontSize: '1.2rem', marginBottom: '2rem'}}>
          {siteConfig.tagline}
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/intro">
          Start Reading →
        </Link>
      </main>
    </Layout>
  );
}
