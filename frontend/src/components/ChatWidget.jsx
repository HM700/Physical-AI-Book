import React, { useState, useEffect, useRef } from 'react';

const ChatWidget = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Function to get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      setSelectedText(selectedText);
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Determine if we have selected text
      const requestBody = {
        user_question: inputValue,
        session_id: sessionId || null
      };

      // Only include selected text if it exists and user wants to use it
      if (selectedText.trim()) {
        const useSelected = window.confirm(`Use selected text for your question?\n\n"${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`);
        if (useSelected) {
          requestBody.selected_text = selectedText;
        }
      }

      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Failed to get response');
      }

      // Update session ID if new one was returned
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        references: data.references || [],
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chat-widget">
      <div className="chat-header">
        <h3>Book Assistant</h3>
        {selectedText && (
          <div className="selected-text-preview">
            <small>Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</small>
          </div>
        )}
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Ask me anything about the Physical AI Book!</p>
            <p>You can also select text on the page and ask questions about it specifically.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div key={message.id} className={`message ${message.sender}`}>
              <div className="message-content">
                <p>{message.text}</p>
                {message.references && message.references.length > 0 && (
                  <div className="references">
                    <strong>References:</strong>
                    <ul>
                      {message.references.map((ref, index) => (
                        <li key={index}>
                          <a href={ref.url} target="_blank" rel="noopener noreferrer">
                            {ref.title} {ref.section && `- ${ref.section}`}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message bot">
            <div className="message-content">
              <p>Thinking...</p>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <div className="chat-input-area">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about the book..."
          rows="3"
        />
        <button
          onClick={sendMessage}
          disabled={!inputValue.trim() || isLoading}
          className="send-button"
        >
          Send
        </button>
      </div>

      <style jsx>{`
        .chat-widget {
          border: 1px solid #ddd;
          border-radius: 8px;
          overflow: hidden;
          display: flex;
          flex-direction: column;
          height: 500px;
          max-width: 100%;
          background: white;
          box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }

        .chat-header {
          background: #4f46e5;
          color: white;
          padding: 12px 16px;
        }

        .chat-header h3 {
          margin: 0;
          font-size: 16px;
        }

        .selected-text-preview {
          font-size: 12px;
          margin-top: 4px;
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 16px;
          display: flex;
          flex-direction: column;
        }

        .welcome-message {
          text-align: center;
          color: #666;
          font-style: italic;
          margin-top: 50px;
        }

        .message {
          margin-bottom: 16px;
          max-width: 80%;
        }

        .user {
          align-self: flex-end;
          text-align: right;
        }

        .bot {
          align-self: flex-start;
        }

        .message-content {
          padding: 12px 16px;
          border-radius: 18px;
          display: inline-block;
        }

        .user .message-content {
          background: #4f46e5;
          color: white;
        }

        .bot .message-content {
          background: #f3f4f6;
          color: #374151;
        }

        .references {
          margin-top: 8px;
          font-size: 14px;
        }

        .references ul {
          margin: 4px 0;
          padding-left: 20px;
        }

        .references a {
          color: #4f46e5;
          text-decoration: none;
        }

        .references a:hover {
          text-decoration: underline;
        }

        .chat-input-area {
          padding: 16px;
          border-top: 1px solid #eee;
          display: flex;
          flex-direction: column;
        }

        textarea {
          width: 100%;
          padding: 12px;
          border: 1px solid #ddd;
          border-radius: 4px;
          resize: vertical;
          margin-bottom: 8px;
          font-family: inherit;
        }

        .send-button {
          align-self: flex-end;
          padding: 8px 16px;
          background: #4f46e5;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
        }

        .send-button:disabled {
          background: #ccc;
          cursor: not-allowed;
        }
      `}</style>
    </div>
  );
};

export default ChatWidget;