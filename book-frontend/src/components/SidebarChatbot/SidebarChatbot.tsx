import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Chatbot from '../Chatbot/Chatbot';
import './SidebarChatbot.css';

const SidebarChatbot: React.FC = () => {
  const [isVisible, setIsVisible] = useState(false);

  const toggleVisibility = () => {
    setIsVisible(!isVisible);
  };

  // Close the chat when clicking outside (only in browser)
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const chatContainer = document.querySelector('.sidebar-chatbot-container');
      if (chatContainer && !chatContainer.contains(event.target as Node)) {
        setIsVisible(false);
      }
    };

    if (isVisible) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isVisible]);

  return (
    <BrowserOnly>
      {() => (
        <div className="sidebar-chatbot">
          <button
            className={`chatbot-toggle ${isVisible ? 'hidden' : ''}`}
            onClick={toggleVisibility}
            aria-label="Open chatbot"
          >
            💬
          </button>
          {isVisible && (
            <div className="sidebar-chatbot-container">
              <div className="sidebar-chatbot-header">
                <h4>Book Assistant</h4>
                <button
                  className="close-button"
                  onClick={() => setIsVisible(false)}
                  aria-label="Close chatbot"
                >
                  ×
                </button>
              </div>
              <div className="sidebar-chatbot-content">
                <Chatbot />
              </div>
            </div>
          )}
        </div>
      )}
    </BrowserOnly>
  );
};

export default SidebarChatbot;