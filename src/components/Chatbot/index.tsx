import React, { useState, useRef, useEffect, forwardRef, useImperativeHandle } from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import rehypeHighlight from 'rehype-highlight';
import AuthModal from '../AuthModal';
import styles from './styles.module.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'ai';
  timestamp: Date;
}

export interface ChatbotRef {
  openWithQuery: (query: string) => void;
}

// Component to render markdown content
const MarkdownMessage: React.FC<{ content: string }> = ({ content }) => {
  return (
    <ReactMarkdown
      remarkPlugins={[remarkGfm]}
      rehypePlugins={[rehypeHighlight as any]}
      components={{
        // Custom styling for code blocks
        code: ({ inline, className, children, ...props }: any) => {
          return inline ? (
            <code className={styles.inlineCode} {...props}>
              {children}
            </code>
          ) : (
            <code className={className} {...props}>
              {children}
            </code>
          );
        },
        // Custom styling for links
        a: ({ children, ...props }) => (
          <a className={styles.markdownLink} target="_blank" rel="noopener noreferrer" {...props}>
            {children}
          </a>
        ),
        // Custom styling for lists
        ul: ({ children, ...props }) => (
          <ul className={styles.markdownList} {...props}>
            {children}
          </ul>
        ),
        ol: ({ children, ...props }) => (
          <ol className={styles.markdownList} {...props}>
            {children}
          </ol>
        ),
        // Custom styling for paragraphs
        p: ({ children, ...props }) => (
          <p className={styles.markdownParagraph} {...props}>
            {children}
          </p>
        ),
        // Custom styling for headings
        h1: ({ children, ...props }) => (
          <h1 className={styles.markdownHeading} {...props}>
            {children}
          </h1>
        ),
        h2: ({ children, ...props }) => (
          <h2 className={styles.markdownHeading} {...props}>
            {children}
          </h2>
        ),
        h3: ({ children, ...props }) => (
          <h3 className={styles.markdownHeading} {...props}>
            {children}
          </h3>
        ),
        // Custom styling for blockquotes
        blockquote: ({ children, ...props }) => (
          <blockquote className={styles.markdownBlockquote} {...props}>
            {children}
          </blockquote>
        ),
      }}
    >
      {content}
    </ReactMarkdown>
  );
};

const Chatbot = forwardRef<ChatbotRef>((props, ref) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics textbook. I can help you with questions about ROS 2, robot simulation, AI-powered navigation, and more. What would you like to learn about?',
      sender: 'ai',
      timestamp: new Date(),
    },
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Check authentication status on mount
  useEffect(() => {
    checkAuthStatus();
  }, []);

  const checkAuthStatus = async () => {
    try {
      const response = await fetch('https://huzaifa1102-better-auth.hf.space/api/auth/session', {
        credentials: 'include',
      });
      if (response.ok) {
        const data = await response.json();
        setIsAuthenticated(!!data.user);
      }
    } catch (error) {
      console.error('Auth check failed:', error);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendQuery();
    }
  };


  // Expose methods to parent components via ref
  useImperativeHandle(ref, () => ({
    openWithQuery: (query: string) => {
      setIsOpen(true);
      setInputValue(query);
      // Auto-send the query after a brief delay to ensure UI is ready
      setTimeout(() => {
        sendQuery(query);
      }, 100);
    },
  }));

  // Refactored send message function to accept optional query parameter
  const sendQuery = async (query?: string) => {
    const messageText = query || inputValue;
    if (!messageText.trim()) return;

    // Check authentication before sending
    if (!isAuthenticated) {
      setShowAuthModal(true);
      return;
    }

    const userMessage: Message = {
      id: Date.now().toString(),
      text: messageText,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsTyping(true);

    try {
      // Call the RAG bot API
      const response = await fetch('https://huzaifa1102-book-rag-bot.hf.space/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: messageText }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Unknown error' }));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      
      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.answer || 'I received your message but couldn\'t generate a response.',
        sender: 'ai',
        timestamp: new Date(),
      };
      
      setMessages((prev) => [...prev, aiMessage]);
    } catch (error) {
      console.error('Error calling RAG bot:', error);
      
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: error instanceof Error 
          ? `Sorry, I encountered an error: ${error.message}. Please make sure the RAG bot server is running on port 8000.`
          : 'Sorry, I encountered an unexpected error. Please try again.',
        sender: 'ai',
        timestamp: new Date(),
      };
      
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsTyping(false);
    }
  };

  return (
    <>
      {/* Floating Chatbot Button */}
      <button
        className={`${styles.chatbotButton} ${isOpen ? styles.chatbotButtonActive : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? (
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            className={styles.icon}
          >
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            className={styles.icon}
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
        <span className={styles.notificationBadge}>1</span>
      </button>

      {/* Chatbot Window */}
      <div className={`${styles.chatbotWindow} ${isOpen ? styles.chatbotWindowOpen : ''}`}>
        {/* Header */}
        <div className={styles.chatbotHeader}>
          <div className={styles.headerContent}>
            <div className={styles.aiAvatar}>
              <svg
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <circle cx="12" cy="12" r="10"></circle>
                <path d="M8 14s1.5 2 4 2 4-2 4-2"></path>
                <line x1="9" y1="9" x2="9.01" y2="9"></line>
                <line x1="15" y1="9" x2="15.01" y2="9"></line>
              </svg>
            </div>
            <div className={styles.headerText}>
              <h3>AI Assistant</h3>
              <p className={styles.status}>
                <span className={styles.statusDot}></span>
                Online
              </p>
            </div>
          </div>
          <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
            {!isAuthenticated && (
              <button 
                onClick={() => setShowAuthModal(true)}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  padding: '6px 12px',
                  borderRadius: '20px',
                  fontSize: '12px',
                  fontWeight: 600,
                  cursor: 'pointer',
                  backdropFilter: 'blur(10px)',
                  transition: 'all 0.2s ease',
                  boxShadow: '0 2px 8px rgba(0,0,0,0.1)'
                }}
                onMouseOver={(e) => {
                  e.currentTarget.style.background = 'rgba(255, 255, 255, 0.3)';
                  e.currentTarget.style.transform = 'translateY(-1px)';
                }}
                onMouseOut={(e) => {
                  e.currentTarget.style.background = 'rgba(255, 255, 255, 0.2)';
                  e.currentTarget.style.transform = 'translateY(0)';
                }}
              >
                Sign In
              </button>
            )}
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chatbot"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>
        </div>

        {/* Messages Container */}
        <div className={styles.messagesContainer}>
          {messages.map((message) => (
            <div
              key={message.id}
              className={`${styles.messageWrapper} ${
                message.sender === 'user' ? styles.userMessageWrapper : styles.aiMessageWrapper
              }`}
            >
              {message.sender === 'ai' && (
                <div className={styles.messageAvatar}>
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  >
                    <circle cx="12" cy="12" r="10"></circle>
                    <path d="M8 14s1.5 2 4 2 4-2 4-2"></path>
                    <line x1="9" y1="9" x2="9.01" y2="9"></line>
                    <line x1="15" y1="9" x2="15.01" y2="9"></line>
                  </svg>
                </div>
              )}
              <div
                className={`${styles.message} ${
                  message.sender === 'user' ? styles.userMessage : styles.aiMessage
                }`}
              >
                {message.sender === 'ai' ? (
                  <MarkdownMessage content={message.text} />
                ) : (
                  message.text
                )}
              </div>
              {message.sender === 'user' && (
                <div className={styles.messageAvatar}>
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  >
                    <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
                    <circle cx="12" cy="7" r="4"></circle>
                  </svg>
                </div>
              )}
            </div>
          ))}

          {/* Typing Indicator */}
          {isTyping && (
            <div className={`${styles.messageWrapper} ${styles.aiMessageWrapper}`}>
              <div className={styles.messageAvatar}>
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <circle cx="12" cy="12" r="10"></circle>
                  <path d="M8 14s1.5 2 4 2 4-2 4-2"></path>
                  <line x1="9" y1="9" x2="9.01" y2="9"></line>
                  <line x1="15" y1="9" x2="15.01" y2="9"></line>
                </svg>
              </div>
              <div className={`${styles.message} ${styles.aiMessage} ${styles.typingIndicator}`}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Auth Prompt for Unauthenticated Users */}
        {!isAuthenticated && (
          <div className={styles.authPrompt}>
            <button className={styles.authPromptButton} onClick={() => setShowAuthModal(true)}>
              Sign In
            </button>
          </div>
        )}

        {/* Input Area */}
        <div className={styles.inputContainer}>
          <input
            ref={inputRef}
            type="text"
            className={styles.input}
            placeholder="Type your message..."
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
          />
          <button
            className={styles.sendButton}
            onClick={() => sendQuery()}
            disabled={!inputValue.trim()}
            aria-label="Send message"
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <line x1="22" y1="2" x2="11" y2="13"></line>
              <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
            </svg>
          </button>
        </div>
      </div>

      {/* Auth Modal */}
      <AuthModal 
        isOpen={showAuthModal} 
        onClose={() => setShowAuthModal(false)}
        onSuccess={() => {
          setIsAuthenticated(true);
          setShowAuthModal(false);
        }}
      />
    </>
  );
});

Chatbot.displayName = 'Chatbot';

export default Chatbot;
