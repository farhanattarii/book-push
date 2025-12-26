import React from 'react';
import clsx from 'clsx';
import styles from './ContentPresentation.module.css';

// Define the ContentPresentation component with enhanced styling
const ContentPresentation = ({children, type = 'default', className = ''}) => {
  const classes = clsx(
    styles.contentPresentation,
    styles[`contentPresentation--${type}`],
    className,
  );

  return (
    <div className={classes}>
      <div className={styles.contentPresentationInner}>
        {children}
      </div>
    </div>
  );
};

export default ContentPresentation;