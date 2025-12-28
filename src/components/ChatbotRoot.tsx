import React from 'react';
import Chatbot from './Chatbot';

// This component will be rendered on all pages
export default function ChatbotRoot(): JSX.Element {
  // Use environment variable or default to localhost for development
  const apiUrl = typeof window !== 'undefined' 
    ? (window as any).CHATBOT_API_URL || 'http://localhost:8000'
    : 'http://localhost:8000';

  return <Chatbot apiUrl={apiUrl} />;
}



