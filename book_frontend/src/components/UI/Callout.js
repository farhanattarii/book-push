import React from 'react';
import clsx from 'clsx';
import styles from './Callout.module.css';

const Callout = ({children, title, type = 'note', icon, className = '', ...props}) => {
  const calloutClasses = clsx(
    styles.callout,
    styles[`callout--${type}`],
    className,
  );

  // Default icons for different types
  const defaultIcons = {
    note: 'ℹ️',
    tip: '💡',
    warning: '⚠️',
    danger: '❌',
    success: '✅',
  };

  const displayIcon = icon || defaultIcons[type] || defaultIcons.note;

  return (
    <div className={calloutClasses} {...props}>
      <div className={styles.calloutHeader}>
        <div className={styles.calloutIcon} aria-hidden="true">
          {displayIcon}
        </div>
        {title && <h4 className={styles.calloutTitle}>{title}</h4>}
      </div>
      <div className={styles.calloutBody}>
        {children}
      </div>
    </div>
  );
};

export default Callout;