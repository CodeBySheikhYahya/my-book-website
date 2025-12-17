import React, { useState, useRef, useEffect } from 'react';
import './Chatbot.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    title: string;
    source: string;
    text: string;
  }>;
}

interface ChatbotProps {
  apiUrl?: string;
}

export default function Chatbot({ apiUrl = 'http://localhost:8000' }: ChatbotProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [sessionId, setSessionId] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Initialize session ID
  useEffect(() => {
    // Get or create session ID from localStorage
    const storedSessionId = localStorage.getItem('chatbot_session_id');
    if (storedSessionId) {
      setSessionId(storedSessionId);
    } else {
      // Will be set when first message is sent
    }
  }, []);

  // Get selected text from page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim()) {
        setSelectedText(selection.toString().trim());
      } else {
        setSelectedText('');
      }
    };

    document.addEventListener('selectionchange', handleSelection);
    return () => document.removeEventListener('selectionchange', handleSelection);
  }, []);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${apiUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: userMessage.content,
          selected_text: selectedText || undefined,
          session_id: sessionId || undefined,
        }),
      });

      const data = await response.json();
      
      // Store session ID if returned
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
        localStorage.setItem('chatbot_session_id', data.session_id);
      }
      
      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer || 'Sorry, I could not generate a response.',
        sources: data.sources || [],
      };
      
      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selection after use
    } catch (error) {
      const errorMessage: Message = {
        role: 'assistant',
        content: 'Error: Could not connect to the chatbot. Make sure the backend server is running.',
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSelectedText('');
    // Keep session ID for history continuity
  };

  return (
    <>
      {/* Chat Button */}
      <button
        className="chatbot-toggle"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className="chatbot-container">
          <div className="chatbot-header">
            <h3>Book Assistant</h3>
            <div className="chatbot-header-actions">
              <button onClick={clearChat} className="chatbot-clear-btn" title="Clear chat">
                🗑️
              </button>
              <button onClick={() => setIsOpen(false)} className="chatbot-close-btn" title="Close">
                ✕
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 && (
              <div className="chatbot-welcome">
                <p>👋 Hi! I'm your book assistant.</p>
                <p>Ask me anything about "Physical AI & Humanoid Robotics"!</p>
                {selectedText && (
                  <div className="chatbot-selected-text">
                    <strong>Selected text:</strong> "{selectedText.substring(0, 100)}..."
                  </div>
                )}
              </div>
            )}
            {messages.map((msg, idx) => (
              <div key={idx} className={`chatbot-message chatbot-message-${msg.role}`}>
                <div className="chatbot-message-content">
                  {msg.content}
                </div>
                {msg.sources && msg.sources.length > 0 && (
                  <div className="chatbot-sources">
                    <strong>Sources:</strong>
                    {msg.sources.map((source, sIdx) => (
                      <div key={sIdx} className="chatbot-source">
                        <span className="chatbot-source-title">{source.title}</span>
                        <span className="chatbot-source-path">{source.source}</span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {loading && (
              <div className="chatbot-message chatbot-message-assistant">
                <div className="chatbot-loading">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-container">
            {selectedText && (
              <div className="chatbot-selected-indicator">
                📎 Using selected text
              </div>
            )}
            <textarea
              className="chatbot-input"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the book..."
              rows={2}
              disabled={loading}
            />
            <button
              className="chatbot-send-btn"
              onClick={sendMessage}
              disabled={loading || !input.trim()}
            >
              {loading ? '⏳' : '➤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
}

