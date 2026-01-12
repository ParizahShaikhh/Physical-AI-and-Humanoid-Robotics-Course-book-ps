import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Chatbot from '../Chatbot/Chatbot';
import './FloatingChatbot.css';

const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <BrowserOnly>
      {() => (
        <div className="floating-chatbot">
          {isOpen ? (
            <div className="chatbot-modal">
              <div className="chatbot-header">
                <h3>Book Assistant</h3>
                <button className="close-btn" onClick={toggleChat}>×</button>
              </div>
              <div className="chatbot-content">
                <Chatbot />
              </div>
            </div>
          ) : (
            <button className="chatbot-trigger" onClick={toggleChat}>
              💬
            </button>
          )}
        </div>
      )}
    </BrowserOnly>
  );
};

export default FloatingChatbot;