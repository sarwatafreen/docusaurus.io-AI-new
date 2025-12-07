import React, { useState, useEffect, useRef } from 'react';
import styles from './Chatbot.module.css';
import { useDoc } from '@docusaurus/useDoc';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const Chatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const { siteConfig } = useDocusaurusContext();
  const location = useLocation();
  const currentDoc = useDoc();

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  const getRelevantContext = (query) => {
    const currentPath = location.pathname;
    const currentTitle = currentDoc?.metadata?.title || '';
    const currentDescription = currentDoc?.metadata?.description || '';

    return `You are a helpful assistant for the Physical AI & Humanoid Robotics course.
    The user is currently viewing: "${currentTitle}" (${currentPath}).
    Document description: "${currentDescription}".
    Provide concise, accurate answers related to robotics, ROS 2, Isaac Sim,
    Navigation, VSLAM, and voice control as covered in the course.
    Base your responses on the course material.`;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get context for RAG
      const context = getRelevantContext(inputValue);

      // In a real implementation, this would call your RAG API
      // For now, we'll simulate a response
      const botResponse = await simulateRAGResponse(inputValue, context);

      const botMessage = {
        id: Date.now() + 1,
        text: botResponse,
        sender: 'bot'
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const simulateRAGResponse = async (query, context) => {
    // Simulate API call delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    // In a real implementation, this would call your backend RAG service
    // which would combine the context with the query to generate a response

    // Simple simulation based on keywords
    const queryLower = query.toLowerCase();

    if (queryLower.includes('install') || queryLower.includes('setup') || queryLower.includes('docker')) {
      return "For installation, please refer to the 'Hardware & Setup Guide' section. You can either use the recommended hardware kit (~$700) with NVIDIA Jetson Orin Nano, Intel RealSense camera, and ReSpeaker microphone, or run everything in Docker for free.";
    }

    if (queryLower.includes('ros') || queryLower.includes('navigation') || queryLower.includes('nav2')) {
      return "Navigation in ROS 2 is handled by the Navigation2 stack. It includes global and local planners, costmap management, and behavior trees. For more details, check Module 3, Lesson 4 on Navigation with Nav2.";
    }

    if (queryLower.includes('voice') || queryLower.includes('whisper') || queryLower.includes('gpt')) {
      return "Voice recognition is implemented using OpenAI's Whisper for speech-to-text and GPT for natural language understanding. The system converts voice commands to structured robot actions as detailed in Module 4.";
    }

    if (queryLower.includes('isaac') || queryLower.includes('simulation')) {
      return "NVIDIA Isaac Sim provides photorealistic simulation for robotics. It's used in Module 3 for training robots with synthetic data and testing perception algorithms.";
    }

    if (queryLower.includes('vslam') || queryLower.includes('mapping')) {
      return "Visual SLAM (VSLAM) enables robots to map their environment while localizing themselves. In this course, we implement ORB-SLAM3 for this purpose. See Module 3, Lesson 3 for implementation details.";
    }

    return `I understand you're asking about "${query}". Based on the context of this course on Physical AI & Humanoid Robotics, the solution typically involves ROS 2 for communication, Isaac Sim for simulation, and AI models for perception. For more specific details, check the relevant module documentation.`;
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const startNewChat = () => {
    setMessages([]);
  };

  if (!isOpen) {
    return (
      <button className={styles.chatbotToggle} onClick={toggleChat}>
        <span>🤖</span> AI Assistant
      </button>
    );
  }

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.chatHeader}>
        <h3>Physical AI Assistant</h3>
        <div className={styles.headerActions}>
          <button onClick={startNewChat} className={styles.newChatButton}>
            New Chat
          </button>
          <button onClick={toggleChat} className={styles.closeButton}>
            ×
          </button>
        </div>
      </div>

      <div className={styles.chatMessages}>
        {messages.length === 0 && (
          <div className={styles.welcomeMessage}>
            <p>Hello! I'm your Physical AI & Robotics assistant.</p>
            <p>Ask me about:</p>
            <ul>
              <li>ROS 2 setup and concepts</li>
              <li>Isaac Sim and simulation</li>
              <li>VSLAM and navigation</li>
              <li>Voice control and GPT integration</li>
              <li>Hardware setup and troubleshooting</li>
            </ul>
          </div>
        )}

        {messages.map((message) => (
          <div
            key={message.id}
            className={`${styles.message} ${styles[message.sender + 'Message']}`}
          >
            {message.text}
          </div>
        ))}

        {isLoading && (
          <div className={`${styles.message} ${styles.botMessage}`}>
            <div className={styles.typingIndicator}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className={styles.chatInputForm}>
        <input
          ref={inputRef}
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask about Physical AI, ROS 2, Isaac Sim..."
          disabled={isLoading}
          className={styles.chatInput}
        />
        <button type="submit" disabled={isLoading} className={styles.sendButton}>
          Send
        </button>
      </form>
    </div>
  );
};

export default Chatbot;
