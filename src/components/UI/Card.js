import React from 'react';
import clsx from 'clsx';
import styles from './Card.module.css';

const Card = ({children, title, description, icon, className = '', variant = 'default', ...props}) => {
  const cardClasses = clsx(
    styles.card,
    styles[`card--${variant}`],
    className,
  );

  return (
    <div className={cardClasses} {...props}>
      <div className={styles.cardHeader}>
        {icon && <div className={styles.cardIcon}>{icon}</div>}
        {title && <h3 className={styles.cardTitle}>{title}</h3>}
      </div>
      <div className={styles.cardBody}>
        {description && <p className={styles.cardDescription}>{description}</p>}
        {children && <div className={styles.cardContent}>{children}</div>}
      </div>
    </div>
  );
};

export default Card;