import React, { useState, useRef, useEffect } from 'react';
import './Chatbot.css';
import ApiService, { QueryResponse } from './apiService';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {useLocation} from '@docusaurus/router';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  sources?: Array<{
    title: string;
    url: string;
  }>;
  confidence?: string;
}

const Chatbot: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your book assistant. Ask me anything about the content.',
      sender: 'bot',
      timestamp: new Date(),
    }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedMode, setSelectedMode] = useState<'full-book' | 'selected-text'>('full-book');
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const location = useLocation();
  const apiService = new ApiService(); // Uses REACT_APP_BACKEND_URL from environment or defaults to localhost

  // Get current page context (title, pathname, etc.)
  const getCurrentPageContext = () => {
    // In a real implementation, you could extract more context from the page
    return {
      pathname: location.pathname,
      title: typeof document !== 'undefined' ? document.title : 'Book Assistant',
      url: typeof window !== 'undefined' ? window.location.href : '',
    };
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputText,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Get current page context
      const pageContext = getCurrentPageContext();
      const contextStr = `Current page: ${pageContext.title}, URL: ${pageContext.url}, Path: ${pageContext.pathname}`;

      // Call the backend API using the service with context
      const response: QueryResponse = await apiService.sendQuery({
        query: inputText,
        mode: selectedMode,
        filters: {},
        top_k: 5,
      }, contextStr);

      // Add bot response
      const botMessage: Message = {
        id: Date.now().toString(),
        text: response.answer || 'Sorry, I couldn\'t process your request.',
        sender: 'bot',
        timestamp: new Date(),
        sources: response.sources?.map(source => ({
          title: source.title,
          url: source.url,
        })) || [],
        confidence: response.confidence_level || 'Medium',
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: Message = {
        id: Date.now().toString(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Book Assistant</h3>
        <div className="mode-selector">
          <label htmlFor="query-mode">Query Mode:</label>
          <select
            id="query-mode"
            value={selectedMode}
            onChange={(e) => setSelectedMode(e.target.value as 'full-book' | 'selected-text')}
            disabled={isLoading}
          >
            <option value="full-book">Full Book</option>
            <option value="selected-text">Selected Text</option>
          </select>
        </div>
      </div>

      <div className="chatbot-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.sender}-message`}
          >
            <div className="message-content">
              <p>{message.text}</p>

              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  <strong>Sources:</strong>
                  <ul>
                    {message.sources.map((source, index) => (
                      <li key={index}>
                        <a href={source.url} target="_blank" rel="noopener noreferrer">
                          {source.title}
                        </a>
                      </li>
                    ))}
                  </ul>
                </div>
              )}

              {message.confidence && (
                <div className="message-confidence">
                  <small>Confidence: {message.confidence}</small>
                </div>
              )}
            </div>
            <div className="message-timestamp">
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chatbot-input-area">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about the book content..."
          disabled={isLoading}
          rows={3}
        />
        <button
          onClick={handleSendMessage}
          disabled={!inputText.trim() || isLoading}
          className="send-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default Chatbot;