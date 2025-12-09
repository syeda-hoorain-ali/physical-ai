import { ChatKit, useChatKit } from '@openai/chatkit-react'
import { useState, useEffect, Activity } from 'react'
import { MessageCircle, X, RefreshCw } from 'lucide-react'

export const ChatWidget = () => {
  const baseUrl = process.env.BASE_URL || "http://127.0.0.1:8000"
  const domainKey = process.env.DOMAIN_KEY || "127.0.0.1"

  const [initialThread, setInitialThread] = useState<string | null>(null)
  const [isReady, setIsReady] = useState(false)
  const [isChatOpen, setIsChatOpen] = useState(false)

  // Load saved thread ID on mount
  useEffect(() => {
    const savedThread = localStorage.getItem('chatkit-thread-id')
    setInitialThread(savedThread)
  }, [])

  const { control, ref } = useChatKit({
    api: {
      url: baseUrl + "/chatkit",
      domainKey: domainKey,
    },
    initialThread: initialThread,
    theme: {
      colorScheme: 'dark',
      color: {
        accent: { primary: 'hsl(174, 72%, 56%)', level: 1 },
        surface: { background: 'hsl(180, 14%, 15%)', foreground: "hsl(180, 20%, 20%)" }
      },
      radius: 'round',
    },
    startScreen: {
      greeting: 'Welcome to Physical AI Assistant!',
    },
    composer: {
      placeholder: 'Ask me anything about robotics...',
      attachments: { enabled: false },
    },
    onThreadChange: ({ threadId }) => {
      console.log('Thread changed:', threadId)
      if (threadId) {
        localStorage.setItem('chatkit-thread-id', threadId)
      }
    },
    onError: ({ error }) => {
      console.error('ChatKit error:', error)
    },
    onReady: () => {
      setIsReady(true)
    },
  })


  return (<>
    {/* Floating Chat Button (bottom-right) */}
    {!isChatOpen && (
      <button
        onClick={() => setIsChatOpen(true)}
        className="fixed bottom-6 right-6 z-50 w-14 h-14 rounded-full bg-linear-to-br from-primary to-secondary flex items-center justify-center shadow-lg shadow-primary/30 transition-all duration-300 ease-out hover:scale-110 hover:shadow-xl hover:shadow-primary/40 animate-pulse-glow cursor-pointer"
        aria-label="Open chat"
      >
        <MessageCircle className="w-6 h-6 text-primary-foreground" />
      </button>
    )}

    {/* Chat Popup */}
    <Activity mode={isChatOpen ? "visible" : "hidden"}>
      {/* Backdrop */}
      <div
        onClick={() => setIsChatOpen(false)}
        className="fixed inset-0 bg-background/60 backdrop-blur-sm z-999"
      />

      {/* Popup Window */}
      <div className="fixed bottom-6 right-6 z-1000 w-105 h-150 max-w-[calc(100vw-3rem)] max-h-[calc(100vh-3rem)] bg-card border border-border rounded-2xl shadow-2xl shadow-primary/20 flex flex-col overflow-hidden animate-slide-in-up">
        {/* Chat Header */}
        <div className="px-4 py-3 bg-muted border-b border-border flex justify-between items-center shrink-0">
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 rounded-full bg-secondary animate-pulse" />
            <span className="text-primary font-semibold text-sm">
              Physical AI Assistant
            </span>
          </div>

          <div className="flex gap-2">
            <button
              onClick={() => {
                localStorage.removeItem('chatkit-thread-id')
              }}
              className="px-3 py-1.5 rounded-lg bg-primary/20 hover:bg-primary/30 text-primary text-xs font-medium flex items-center gap-1.5 transition-colors duration-200"
            >
              <RefreshCw className="w-3 h-3" />
              New Chat
            </button>
            <button
              onClick={() => setIsChatOpen(false)}
              className="p-1.5 rounded-lg text-muted-foreground hover:text-foreground hover:bg-muted transition-colors duration-200"
              aria-label="Close chat"
            >
              <X className="w-4 h-4" />
            </button>
          </div>
        </div>

        {/* Chat Content */}
        <div className="flex-1 overflow-hidden chatkit-container">
          <Activity mode={isReady ? "hidden" : "visible"}>
            <div className="h-full w-full flex items-center justify-center text-muted-foreground text-sm">
              Connecting to assistant...
            </div>
          </Activity>
          <ChatKit control={control} ref={ref} className="h-full w-full" />
        </div>
      </div>
    </Activity>
  </>);
}

export default ChatWidget
