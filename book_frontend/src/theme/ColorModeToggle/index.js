import React from 'react';
import clsx from 'clsx';
import {useColorMode, useThemeConfig} from '@docusaurus/theme-common';
import useIsBrowser from '@docusaurus/useIsBrowser';
import styles from './styles.module.css';

import Dark from '@site/static/img/moon.svg';
import Light from '@site/static/img/sun.svg';

function IconToggle({icon: Icon, onClick, title}) {
  return (
    <button
      className={clsx(
        'clean-btn',
        styles.iconButton,
      )}
      onClick={onClick}
      title={title}
      aria-label={title}>
      <Icon className={styles.icon} />
    </button>
  );
}

export default function ColorModeToggle({className}) {
  const isBrowser = useIsBrowser();
  const {colorMode, setColorMode} = useColorMode();
  const {disableSwitch} = useThemeConfig().colorMode;
  const isDarkTheme = colorMode === 'dark';

  const switchColorMode = (newColorMode) => {
    setColorMode(newColorMode);
  };

  // On SSR, do nothing
  if (!isBrowser) {
    return <div className={clsx(styles.colorModeToggle, className)} />;
  }

  if (disableSwitch) {
    return (
      <div className={clsx(styles.colorModeToggle, className)}>
        <div className={styles.iconWrapper}>
          {isDarkTheme ? <Dark /> : <Light />}
        </div>
      </div>
    );
  }

  return (
    <div className={clsx(styles.colorModeToggle, className)}>
      <IconToggle
        icon={Light}
        onClick={() => switchColorMode('light')}
        title="Switch to light mode"
      />
      <IconToggle
        icon={Dark}
        onClick={() => switchColorMode('dark')}
        title="Switch to dark mode"
      />
    </div>
  );
}