import ChatWidget from '@site/src/components/chat-widget/chat-widget';

// Default React component for the root App
export default function Root({ children }) {
  return (
    <>
      <script src="https://cdn.platform.openai.com/deployments/chatkit/chatkit.js" async />
      {children}
      <ChatWidget />
    </>
  );
}
