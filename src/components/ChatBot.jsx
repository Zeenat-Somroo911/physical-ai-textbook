import React, { useState, useEffect, useRef, useCallback } from 'react';
import clsx from 'clsx';
import styles from './ChatBot.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

const ChatBot = () => {
  const { siteConfig } = useDocusaurusContext();
  // Use the URL from docusaurus.config.js (can be set via env var)
  const API_URL = siteConfig.customFields?.chatbotUrl || 'http://localhost:8000';
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [conversationId, setConversationId] = useState(null);
  const [sources, setSources] = useState([]);
  const [showWelcome, setShowWelcome] = useState(true);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const chatWindowRef = useRef(null);

  // Hide welcome message after first interaction
  useEffect(() => {
    if (messages.length > 0) {
      setShowWelcome(false);
    }
  }, [messages]);

  // Scroll to bottom when messages change
  const scrollToBottom = useCallback(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, []);

  useEffect(() => {
    if (isOpen) {
      scrollToBottom();
      inputRef.current?.focus();
    }
  }, [messages, isOpen, scrollToBottom]);

  // Handle text selection on page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      if (selectedText && selectedText.length > 10) {
        setSelectedText(selectedText);

        // Show notification that text is selected
        if (isOpen) {
          // Text is already selected, user can ask about it
        }
      } else {
        setSelectedText('');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [isOpen]);

  // Load conversation history from localStorage
  useEffect(() => {
    const savedConversationId = localStorage.getItem('chatbot_conversation_id');
    const savedMessages = localStorage.getItem('chatbot_messages');

    if (savedConversationId) {
      setConversationId(savedConversationId);
    }

    if (savedMessages) {
      try {
        setMessages(JSON.parse(savedMessages));
      } catch (e) {
        console.error('Error loading saved messages:', e);
      }
    }
  }, []);

  // Save messages to localStorage
  useEffect(() => {
    if (messages.length > 0) {
      localStorage.setItem('chatbot_messages', JSON.stringify(messages));
    }
  }, [messages]);

  const sendMessage = async (messageText = null) => {
    const textToSend = messageText || inputValue.trim();

    if (!textToSend) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: textToSend,
      timestamp: new Date().toISOString(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: textToSend,
          conversation_id: conversationId,
          selected_text: selectedText || undefined,
          use_rag: true,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Update conversation ID
      if (data.conversation_id && data.conversation_id !== conversationId) {
        setConversationId(data.conversation_id);
        localStorage.setItem('chatbot_conversation_id', data.conversation_id);
      }

      // Add assistant message
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response,
        timestamp: data.timestamp,
        sources: data.sources || [],
        tokens_used: data.tokens_used,
      };

      setMessages((prev) => [...prev, assistantMessage]);
      setSources(data.sources || []);
      setSelectedText(''); // Clear selected text after use

    } catch (err) {
      console.error('Error sending message:', err);
      setError(err.message || 'Failed to send message. Please try again.');

      // Add error message
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again later.',
        timestamp: new Date().toISOString(),
        isError: true,
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage();
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearChat = () => {
    setMessages([]);
    setConversationId(null);
    setSources([]);
    setSelectedText('');
    localStorage.removeItem('chatbot_messages');
    localStorage.removeItem('chatbot_conversation_id');
  };

  const askAboutSelection = () => {
    if (selectedText) {
      const question = `Can you explain this: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"?`;
      sendMessage(question);
    }
  };

  return (
    <>
      {/* Floating Chat Button - Hidden when chat is open */}
      {!isOpen && (
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
        >
          {/* Always show chat icon - close button is in header */}
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
          {messages.length > 0 && (
            <span className={styles.notificationBadge}>{messages.length}</span>
          )}
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow} ref={chatWindowRef}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.chatHeaderContent}>
              <div className={styles.chatHeaderIcon}>🤖</div>
              <div>
                <h3 className={styles.chatHeaderTitle}>AI Assistant</h3>
                <p className={styles.chatHeaderSubtitle}>Ask me anything about the textbook</p>
              </div>
            </div>
            <div className={styles.chatHeaderActions}>
              {messages.length > 0 && (
                <button
                  className={styles.clearButton}
                  onClick={clearChat}
                  aria-label="Clear chat"
                  title="Clear chat"
                >
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <polyline points="3 6 5 6 21 6"></polyline>
                    <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                  </svg>
                </button>
              )}
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close chat"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <line x1="18" y1="6" x2="6" y2="18"></line>
                  <line x1="6" y1="6" x2="18" y2="18"></line>
                </svg>
              </button>
            </div>
          </div>

          {/* Selected Text Banner */}
          {selectedText && (
            <div className={styles.selectedTextBanner}>
              <div className={styles.selectedTextContent}>
                <span className={styles.selectedTextLabel}>Selected:</span>
                <span className={styles.selectedTextPreview}>
                  {selectedText.substring(0, 80)}
                  {selectedText.length > 80 ? '...' : ''}
                </span>
              </div>
              <button
                className={styles.askButton}
                onClick={askAboutSelection}
                disabled={isLoading}
              >
                Ask about this
              </button>
            </div>
          )}

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {showWelcome && messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <div className={styles.welcomeIcon}>👋</div>
                <h3>Welcome! I am your reading assistant</h3>
                <p>I can help you understand Physical AI & Humanoid Robotics concepts. Ask me anything!</p>
              </div>
            )}
            {messages.length === 0 && !showWelcome ? (
              <div className={styles.emptyState}>
                <div className={styles.emptyStateIcon}>💬</div>
                <h4 className={styles.emptyStateTitle}>Start a conversation</h4>
                <p className={styles.emptyStateText}>
                  Ask me anything about Physical AI & Humanoid Robotics!
                </p>
                <div className={styles.suggestions}>
                  <button
                    className={styles.suggestionButton}
                    onClick={() => sendMessage('What is ROS 2?')}
                  >
                    What is ROS 2?
                  </button>
                  <button
                    className={styles.suggestionButton}
                    onClick={() => sendMessage('How do I get started?')}
                  >
                    How do I get started?
                  </button>
                  <button
                    className={styles.suggestionButton}
                    onClick={() => sendMessage('Explain VLA systems')}
                  >
                    Explain VLA systems
                  </button>
                </div>
              </div>
            ) : (
              <>
                {messages.map((message) => (
                  <div
                    key={message.id}
                    className={clsx(
                      styles.message,
                      styles[`message${message.role === 'user' ? 'User' : 'Assistant'}`],
                      message.isError && styles.messageError
                    )}
                  >
                    <div className={styles.messageAvatar}>
                      {message.role === 'user' ? '👤' : '🤖'}
                    </div>
                    <div className={styles.messageContent}>
                      <div className={styles.messageText}>
                        {message.role === 'assistant' ? (
                          <ReactMarkdown
                            remarkPlugins={[remarkGfm]}
                            components={{
                              // Custom components to ensure proper styling
                              h1: ({ node, ...props }) => <h1 className={styles.markdownH1} {...props} />,
                              h2: ({ node, ...props }) => <h2 className={styles.markdownH2} {...props} />,
                              h3: ({ node, ...props }) => <h3 className={styles.markdownH3} {...props} />,
                              p: ({ node, ...props }) => <p className={styles.markdownP} {...props} />,
                              ul: ({ node, ...props }) => <ul className={styles.markdownUl} {...props} />,
                              ol: ({ node, ...props }) => <ol className={styles.markdownOl} {...props} />,
                              li: ({ node, ...props }) => <li className={styles.markdownLi} {...props} />,
                              strong: ({ node, ...props }) => <strong className={styles.markdownStrong} {...props} />,
                              code: ({ node, inline, ...props }) =>
                                inline ?
                                  <code className={styles.markdownInlineCode} {...props} /> :
                                  <code className={styles.markdownCode} {...props} />,
                            }}
                          >
                            {message.content}
                          </ReactMarkdown>
                        ) : (
                          message.content
                        )}
                      </div>
                      {message.sources && message.sources.length > 0 && (
                        <div className={styles.messageSources}>
                          <span className={styles.sourcesLabel}>Sources:</span>
                          {message.sources.map((source, idx) => (
                            <a
                              key={idx}
                              href={source}
                              className={styles.sourceLink}
                              target="_blank"
                              rel="noopener noreferrer"
                            >
                              {source.split('/').pop()}
                            </a>
                          ))}
                        </div>
                      )}
                      <div className={styles.messageTime}>
                        {new Date(message.timestamp).toLocaleTimeString([], {
                          hour: '2-digit',
                          minute: '2-digit',
                        })}
                      </div>
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className={clsx(styles.message, styles.messageAssistant)}>
                    <div className={styles.messageAvatar}>🤖</div>
                    <div className={styles.messageContent}>
                      <div className={styles.typingIndicator}>
                        <span></span>
                        <span></span>
                        <span></span>
                      </div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </>
            )}
          </div>

          {/* Input Area */}
          <form className={styles.inputContainer} onSubmit={handleSubmit}>
            <div className={styles.inputWrapper}>
              <textarea
                ref={inputRef}
                className={styles.input}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question..."
                rows={1}
                disabled={isLoading}
              />
              <button
                type="submit"
                className={styles.sendButton}
                disabled={!inputValue.trim() || isLoading}
                aria-label="Send message"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </div>
            {error && (
              <div className={styles.errorMessage}>
                {error}
              </div>
            )}
          </form>
        </div>
      )}
    </>
  );
}

export default ChatBot;

