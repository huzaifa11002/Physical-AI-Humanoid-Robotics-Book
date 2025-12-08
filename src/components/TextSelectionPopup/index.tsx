import { useEffect, useState } from 'react';
import styles from './styles.module.css';

interface TextSelectionPopupProps {
  onAskAI: (selectedText: string) => void;
}

export default function TextSelectionPopup({ onAskAI }: TextSelectionPopupProps) {
  const [selectedText, setSelectedText] = useState('');
  const [popupPosition, setPopupPosition] = useState<{ top: number; left: number } | null>(null);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          // Position the popup above the selected text
          setPopupPosition({
            top: rect.top + window.scrollY - 50,
            left: rect.left + window.scrollX + rect.width / 2,
          });
          setSelectedText(text);
        }
      } else {
        // Clear popup if no text is selected
        setPopupPosition(null);
        setSelectedText('');
      }
    };

    // Listen for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const handleAskAI = () => {
    if (selectedText) {
      onAskAI(selectedText);
      // Clear selection and popup
      window.getSelection()?.removeAllRanges();
      setPopupPosition(null);
      setSelectedText('');
    }
  };

  if (!popupPosition || !selectedText) {
    return null;
  }

  return (
    <div
      className={styles.selectionPopup}
      style={{
        top: `${popupPosition.top}px`,
        left: `${popupPosition.left}px`,
      }}
    >
      <button
        className={styles.askAIButton}
        onClick={handleAskAI}
        aria-label="Ask AI about selected text"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className={styles.buttonIcon}
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          <path d="M8 10h.01M12 10h.01M16 10h.01"></path>
        </svg>
        Ask AI
      </button>
    </div>
  );
}
