import React, {useState, useEffect} from 'react';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import {IoMenu, IoClose} from 'react-icons/io5';
import clsx from 'clsx';
import styles from './Navigation.module.css';

const Navigation = ({items = [], logo = null, title = ''}) => {
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const location = useLocation();

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen);
  };

  const closeMenu = () => {
    setIsMenuOpen(false);
  };

  // Accessibility: Close menu when route changes
  useEffect(() => {
    setIsMenuOpen(false);
  }, [location.pathname]);

  return (
    <nav className={styles.navigation} role="navigation" aria-label="Main navigation">
      <div className={styles.navigationContainer}>
        {/* Logo and title */}
        <div className={styles.navBrand}>
          {logo && (
            <Link to="/" className={styles.navLogo}>
              <img src={logo} alt={title} className={styles.logoImg} />
            </Link>
          )}
          <Link to="/" className={styles.navTitle}>
            {title}
          </Link>
        </div>

        {/* Desktop navigation */}
        <div className={styles.navDesktop}>
          <ul className={styles.navList} role="menubar">
            {items.map((item, index) => (
              <li key={index} className={styles.navItem} role="none">
                {item.type === 'link' ? (
                  <Link
                    to={item.href}
                    className={clsx(
                      styles.navLink,
                      location.pathname === item.href && styles.navLinkActive
                    )}
                    onClick={closeMenu}
                    role="menuitem"
                    tabIndex={isMenuOpen ? -1 : 0}>
                    {item.label}
                  </Link>
                ) : (
                  <span
                    className={styles.navSeparator}
                    role="separator"
                    aria-label={item.label}>
                    {item.label}
                  </span>
                )}
              </li>
            ))}
          </ul>
        </div>

        {/* Mobile menu button */}
        <div className={styles.navMobileToggle}>
          <button
            className={styles.navToggleButton}
            onClick={toggleMenu}
            aria-label={isMenuOpen ? "Close menu" : "Open menu"}
            aria-expanded={isMenuOpen}
            aria-controls="mobile-menu"
            role="button"
          >
            {isMenuOpen ? <IoClose size={24} aria-label="Close icon" /> : <IoMenu size={24} aria-label="Menu icon" />}
          </button>
        </div>
      </div>

      {/* Mobile menu */}
      {isMenuOpen && (
        <div
          className={styles.navMobileMenu}
          id="mobile-menu"
          role="menu"
          aria-label="Mobile navigation menu">
          <ul className={styles.navMobileList} role="menubar">
            {items.map((item, index) => (
              <li key={index} className={styles.navMobileItem} role="none">
                {item.type === 'link' ? (
                  <Link
                    to={item.href}
                    className={clsx(
                      styles.navMobileLink,
                      location.pathname === item.href && styles.navMobileLinkActive
                    )}
                    onClick={closeMenu}
                    role="menuitem"
                    tabIndex={0}>
                    {item.label}
                  </Link>
                ) : (
                  <span
                    className={styles.navMobileSeparator}
                    role="separator"
                    aria-label={item.label}>
                    {item.label}
                  </span>
                )}
              </li>
            ))}
          </ul>
        </div>
      )}
    </nav>
  );
};

export default Navigation;