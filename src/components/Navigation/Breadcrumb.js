import React from 'react';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import clsx from 'clsx';
import styles from './Breadcrumb.module.css';

const Breadcrumb = ({additionalItems = []}) => {
  const location = useLocation();
  const pathSegments = location.pathname
    .split('/')
    .filter(segment => segment !== '')
    .map(segment => ({
      name: segment
        .replace(/-/g, ' ')
        .replace(/\b\w/g, l => l.toUpperCase()),
      path: `/${segment}`
    }));

  // Create breadcrumb items
  const breadcrumbItems = pathSegments.map((segment, index) => {
    const path = pathSegments.slice(0, index + 1).map(s => s.path).join('');
    return {
      name: segment.name,
      path: path
    };
  });

  // Add any additional items
  const allItems = [...breadcrumbItems, ...additionalItems];

  if (allItems.length === 0) {
    return null;
  }

  return (
    <nav className={styles.breadcrumb} aria-label="Breadcrumb">
      <ol className={styles.breadcrumbList}>
        <li className={styles.breadcrumbItem}>
          <Link to="/" className={styles.breadcrumbLink}>
            Home
          </Link>
        </li>
        {allItems.map((item, index) => (
          <React.Fragment key={index}>
            <li className={styles.breadcrumbSeparator} aria-hidden="true">
              /
            </li>
            <li className={styles.breadcrumbItem}>
              {index === allItems.length - 1 ? (
                <span className={styles.breadcrumbCurrent}>
                  {item.name}
                </span>
              ) : (
                <Link
                  to={item.path}
                  className={styles.breadcrumbLink}
                >
                  {item.name}
                </Link>
              )}
            </li>
          </React.Fragment>
        ))}
      </ol>
    </nav>
  );
};

export default Breadcrumb;