# Chatbot Component

A React chatbot component that integrates with the RAG Agent API to provide answers to questions about book content.

## Features

- Real-time chat interface with message history
- Support for both full-book and selected-text query modes
- Display of source citations and confidence levels
- Loading indicators and error handling
- Responsive design for different screen sizes

## Files

- `Chatbot.tsx` - Main React component implementation
- `Chatbot.css` - Styling for the chatbot component
- `apiService.ts` - API service for communicating with the backend

## API Integration

The chatbot communicates with the backend API at `http://localhost:8000` using the following endpoints:

- `POST /api/chat` - For general book content queries
- `POST /api/selected-text` - For queries based on selected text context (not yet implemented in UI)
- `GET /api/health` - For system status monitoring
- `GET /api/metrics` - For performance metrics

## Usage

The component can be imported and used in any React application:

```tsx
import Chatbot from './components/Chatbot/Chatbot';

function App() {
  return (
    <div className="app">
      <Chatbot />
    </div>
  );
}
```

## Configuration

The API base URL can be configured by changing the URL in the `apiService` instantiation in `Chatbot.tsx`:

```ts
const apiService = new ApiService('http://localhost:8000'); // Change this URL as needed
```

## Dependencies

- React (>=17.0.0)
- TypeScript (>=4.0.0)

## Error Handling

The component handles various error scenarios:

- Network errors when communicating with the API
- Invalid responses from the backend
- Loading states during API requests
- User-friendly error messages