import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot/Chatbot';
import FloatingChatbot from '../components/FloatingChatbot/FloatingChatbot';
import './Chat.css';

const ChatPage: React.FC = () => {
  return (
    <Layout title="Book Assistant Chat" description="Chat with the book assistant">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Book Assistant</h1>
            <p>Ask questions about the book content and get answers grounded in the text.</p>

            <div className="chatbot-wrapper">
              <Chatbot />
            </div>

            <div className="chatbot-info">
              <h3>How to use this assistant:</h3>
              <ul>
                <li>Ask questions about ROS 2, robotics, AI, or other topics covered in the course</li>
                <li>Switch between "Full Book" and "Selected Text" modes depending on your query</li>
                <li>Responses include source citations and confidence levels</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
      <FloatingChatbot />
    </Layout>
  );
};

export default ChatPage;