import BrowserOnly from '@docusaurus/BrowserOnly';

export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          const ChatbotWithSelection = require('@site/src/components/ChatbotWithSelection').default;
          return <ChatbotWithSelection />;
        }}
      </BrowserOnly>
    </>
  );
}
