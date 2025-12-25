import React from 'react';
import {motion} from 'framer-motion';

const AnimatedCard = ({children, className = '', delay = 0, ...props}) => {
  const containerVariants = {
    hidden: { opacity: 0, y: 20 },
    visible: {
      opacity: 1,
      y: 0,
      transition: {
        duration: 0.5,
        delay: delay,
        ease: "easeOut"
      }
    }
  };

  return (
    <motion.div
      className={className}
      initial="hidden"
      whileInView="visible"
      viewport={{ once: true, amount: 0.1 }}
      variants={containerVariants}
      {...props}
    >
      {children}
    </motion.div>
  );
};

export default AnimatedCard;