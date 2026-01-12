import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import Chatbot from './Chatbot';

// Mock the apiService
jest.mock('./apiService', () => ({
  __esModule: true,
  default: jest.fn().mockImplementation(() => ({
    sendQuery: jest.fn().mockResolvedValue({
      answer: 'Test response',
      sources: [{ title: 'Test Source', url: 'http://example.com', score: 0.9 }],
      confidence_level: 'High',
      confidence_score: 0.9,
      timestamp: new Date().toISOString(),
    }),
    getHealth: jest.fn().mockResolvedValue({ status: 'healthy' }),
    getMetrics: jest.fn().mockResolvedValue({}),
  })),
}));

describe('Chatbot', () => {
  beforeEach(() => {
    // Clear all mocks before each test
    jest.clearAllMocks();
  });

  it('renders initial bot message', () => {
    render(<Chatbot />);

    expect(screen.getByText(/Hello! I'm your book assistant/i)).toBeInTheDocument();
  });

  it('allows user to type and send a message', async () => {
    render(<Chatbot />);

    const input = screen.getByPlaceholderText(/Ask a question about the book content/i);
    const sendButton = screen.getByText('Send');

    fireEvent.change(input, { target: { value: 'Test question' } });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(screen.getByText('Test question')).toBeInTheDocument();
    });
  });

  it('disables send button when input is empty', () => {
    render(<Chatbot />);

    const sendButton = screen.getByText('Send');
    expect(sendButton).toBeDisabled();
  });

  it('enables send button when input has text', () => {
    render(<Chatbot />);

    const input = screen.getByPlaceholderText(/Ask a question about the book content/i);
    fireEvent.change(input, { target: { value: 'Test' } });

    const sendButton = screen.getByText('Send');
    expect(sendButton).not.toBeDisabled();
  });

  it('displays loading state when sending message', async () => {
    render(<Chatbot />);

    const input = screen.getByPlaceholderText(/Ask a question about the book content/i);
    fireEvent.change(input, { target: { value: 'Test question' } });

    const sendButton = screen.getByText('Send');
    fireEvent.click(sendButton);

    // Check for loading state
    expect(screen.getByText('Sending...')).toBeInTheDocument();
  });

  it('allows mode selection', () => {
    render(<Chatbot />);

    const modeSelect = screen.getByRole('combobox');
    expect(modeSelect).toBeInTheDocument();

    // Check that both options are available
    expect(screen.getByText('Full Book')).toBeInTheDocument();
    expect(screen.getByText('Selected Text')).toBeInTheDocument();
  });
});