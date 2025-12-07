import React, { useState, useRef } from 'react';
import Layout from '@theme/Layout';
import './chat.css'; // Optional: for custom styling

const ChatPage = () => {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [useRag, setUseRag] = useState(true);
  const [selectedText, setSelectedText] = useState('');
  const [showTextInput, setShowTextInput] = useState(false);
  const textRef = useRef(null);

  const handleSendMessage = async () => {
    if (!inputMessage.trim()) return;

    // Add user message to the chat
    const userMessage = {
      text: inputMessage,
      sender: 'user',
      selectedText: selectedText
    };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Send the message to the backend
      const response = await fetch('http://127.0.0.1:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputMessage,
          use_rag: useRag,
          selected_text: selectedText
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to the chat
      const botMessage = {
        text: data.reply,
        sender: 'bot',
        useRag: data.use_rag,
        selectedTextUsed: data.selected_text_used
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = { text: 'Sorry, there was an error processing your request.', sender: 'bot' };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setInputMessage('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleTextSelection = () => {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    if (selectedText) {
      setSelectedText(selectedText);
      setShowTextInput(true);
    } else {
      setSelectedText('');
    }
  };

  const clearSelection = () => {
    setSelectedText('');
    setShowTextInput(false);
    if (window.getSelection) {
      window.getSelection().removeAllRanges();
    }
  };

  return (
    <Layout
      title="AI Chatbot"
      description="Chat with our AI assistant powered by Google's Gemini with RAG capabilities"
    >
      <div className="chat-container">
        <div className="chat-header">
          <h1>AI Chatbot</h1>
          <p>Powered by Google's Gemini with RAG capabilities</p>

          <div className="chat-options">
            <label>
              <input
                type="checkbox"
                checked={useRag}
                onChange={(e) => setUseRag(e.target.checked)}
              />
              Use RAG (search book content)
            </label>

            {selectedText && (
              <div className="selected-text-preview">
                <p><strong>Selected for context:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
                <button onClick={clearSelection} className="clear-selection">Clear</button>
              </div>
            )}
          </div>
        </div>

        <div className="book-content" ref={textRef} onMouseUp={handleTextSelection}>
          <h3>Book Content</h3>
          <p>Select text from this area to provide specific context to the AI</p>
          <p>This is where your book content would appear. You can select text to ask specific questions about it.</p>
          <p>Try highlighting this text and then typing a question below to see how the RAG system works!</p>
        </div>

        <div className="chat-messages">
          {messages.map((msg, index) => (
            <div key={index} className={`message ${msg.sender}`}>
              <div className="message-text">{msg.text}</div>
              {msg.selectedTextUsed && (
                <div className="message-context">Using selected text as context</div>
              )}
            </div>
          ))}
          {isLoading && (
            <div className="message bot">
              <div className="message-text">Thinking...</div>
            </div>
          )}
        </div>

        {showTextInput && (
          <div className="selected-text-input">
            <p>Selected text: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
            <button onClick={clearSelection} className="clear-selection">Clear Selection</button>
          </div>
        )}

        <div className="chat-input-area">
          <textarea
            value={inputMessage}
            onChange={(e) => setInputMessage(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your message here..."
            rows="3"
          />
          <button
            onClick={handleSendMessage}
            disabled={isLoading || !inputMessage.trim()}
            className="send-button"
          >
            Send
          </button>
        </div>
      </div>
    </Layout>
  );
};

export default ChatPage;