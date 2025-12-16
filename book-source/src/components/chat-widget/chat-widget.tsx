import { ChatKit, useChatKit } from '@openai/chatkit-react'
import { useState, useEffect, Activity } from 'react'
import { MessageCircle, X, RefreshCw } from 'lucide-react'
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'

export const ChatWidget = () => {

  const { siteConfig: { customFields } } = useDocusaurusContext()
  const baseUrl = (customFields.apiBaseUrl as string) || "http://127.0.0.1:8000"
  const domainKey = (customFields.domainKey as string) || "127.0.0.1"

  const [initialThread, setInitialThread] = useState<string | null>(null)
  const [isReady, setIsReady] = useState(false)
  const [isChatOpen, setIsChatOpen] = useState(false)
  const [selectedText, setSelectedText] = useState<string | null>(null)
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0, width: 0, height: 0 })
  const [contextText, setContextText] = useState<string | null>(null)

  // Load saved thread ID on mount
  useEffect(() => {
    const savedThread = localStorage.getItem('chatkit-thread-id')
    setInitialThread(savedThread)
  }, [])

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection()
      if (selection && selection.toString().trim() !== '') {
        // Check if the selection is within the chat component to avoid showing the button there
        const anchorNode = selection.anchorNode;
        let isWithinChat = false;

        if (anchorNode) {
          let parentNode = anchorNode.parentElement;
          while (parentNode) {
            if (parentNode.classList && parentNode.classList.contains('chatkit-container')) {
              isWithinChat = true;
              break;
            }
            parentNode = parentNode.parentElement;
          }
        }

        if (!isWithinChat) {
          const range = selection.getRangeAt(0)
          const rect = range.getBoundingClientRect()
          setSelectionPosition({ x: rect.left, y: rect.top - 10, width: rect.width, height: rect.height })
          setSelectedText(selection.toString())
        } else {
          setSelectedText(null)
        }
      } else {
        setSelectedText(null)
      }
    }

    document.addEventListener('mouseup', handleSelection)
    document.addEventListener('keyup', handleSelection)

    return () => {
      document.removeEventListener('mouseup', handleSelection)
      document.removeEventListener('keyup', handleSelection)
    }
  }, [])

  const { control, ref, fetchUpdates, focusComposer, sendCustomAction, sendUserMessage, setComposerValue, setThreadId } = useChatKit({
    api: {
      url: baseUrl + "/chatkit",
      domainKey: domainKey,
      fetch: (url: string, options: RequestInit) => {
        // Get current page information
        const currentPagePath = window.location.pathname;
        const pageHeading = document.querySelector('h1')?.textContent?.trim() || 'No heading found';

        // Add custom headers with page context
        const customOptions = {
          ...options,
          headers: {
            ...options.headers,
            'X-Page-Path': currentPagePath,
            'X-Page-Heading': pageHeading,
            'X-Selected-Text': contextText,
            'X-Context-Source': 'chat-widget',
          }
        };

        return fetch(url, customOptions);
      }
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


  const handleAskWithAI = () => {
    if (selectedText) {
      // Set the context text and open the chat
      setContextText(selectedText);
      setIsChatOpen(true);
      // Clear the selection after handling
      setSelectedText(null);
    }
  };

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

    {/* Text Selection Ask with AI Button */}
    {selectedText && !isChatOpen && (
      <div
        className="fixed z-50 -translate-x-1/2 -translate-y-full"
        style={{
          left: selectionPosition.x + (selectionPosition.width / 2) + "px",
          top: selectionPosition.y,
        }}
      >
        <button
          onClick={handleAskWithAI}
          className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-linear-to-r from-primary to-secondary text-primary-foreground text-sm font-medium shadow-lg shadow-primary/30 cursor-pointer transition-all duration-200 hover:scale-105 hover:shadow-xl hover:shadow-primary/40 animate-scale-in">
          <MessageCircle className="w-4 h-4" />
          <span>Ask AI</span>
        </button>
      </div>
    )}

    {/* Chat Popup */}
    <Activity mode={isChatOpen ? "visible" : "hidden"}>
      {/* Backdrop */}
      <div
        onClick={() => setIsChatOpen(false)}
        className="fixed inset-0 bg-background/30 z-999"
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
                setThreadId(null)
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

        {/* #chatkit-footer */}
        {/* Chat Content */}
        <div className="flex-1 overflow-hidden chatkit-container flex flex-col">
          {contextText && (
            <div className="bg-muted border-b border-border p-3 text-sm flex items-start gap-2">
              <div className="flex-1">
                <p className="font-medium text-foreground mb-1">Context from page:</p>
                <p className="text-muted-foreground line-clamp-2">{contextText}</p>
              </div>
              <button
                onClick={() => setContextText(null)}
                className="text-muted-foreground hover:text-foreground p-1 rounded-full hover:bg-accent"
                aria-label="Remove context"
              >
                <X className="w-4 h-4" />
              </button>
            </div>
          )}
          <Activity mode={isReady ? "hidden" : "visible"}>
            <div className="h-full w-full flex items-center justify-center text-muted-foreground text-sm">
              Connecting to assistant...
            </div>
          </Activity>
          <ChatKit control={control} ref={ref} className="flex-1" />
        </div>
      </div>
    </Activity>
  </>);
}

export default ChatWidget
