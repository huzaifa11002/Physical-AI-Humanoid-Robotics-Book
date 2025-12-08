import React, { useRef } from 'react';
import Chatbot, { ChatbotRef } from '../Chatbot';
import TextSelectionPopup from '../TextSelectionPopup';

export default function ChatbotWithSelection() {
  const chatbotRef = useRef<ChatbotRef>(null);

  const handleAskAI = (selectedText: string) => {
    // Format the query to include context
    const query = `Explain this: "${selectedText}"`;
    chatbotRef.current?.openWithQuery(query);
  };

  return (
    <>
      <TextSelectionPopup onAskAI={handleAskAI} />
      <Chatbot ref={chatbotRef} />
    </>
  );
}
