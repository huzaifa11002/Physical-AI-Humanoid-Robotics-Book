# Chatbot Component

A modern, responsive chatbot UI component integrated with the Physical AI & Humanoid Robotics RAG bot backend.

## Features

- 🎨 Modern, premium UI design with smooth animations
- 💬 Real-time chat interface with user and AI avatars
- 🔄 Loading indicator while AI is generating responses
- 📱 Fully responsive (desktop, tablet, mobile)
- 🌓 Dark mode support
- 🤖 **Integrated with RAG bot backend** for intelligent responses about Physical AI & Humanoid Robotics

## Setup & Running

### Prerequisites

1. **RAG Bot Backend** must be running on `http://localhost:8000`
2. **Docusaurus Frontend** must be running on `http://localhost:3000`

### Step 1: Start the RAG Bot Backend

Navigate to the `rag-bot` directory and start the FastAPI server:

```bash
cd rag-bot

# Activate virtual environment (if using one)
# On Windows:
.venv\Scripts\activate
# On macOS/Linux:
# source .venv/bin/activate

# Run the FastAPI server
python main.py
# OR
uvicorn main:app --reload --port 8000
```

The RAG bot API should now be running at `http://localhost:8000`

### Step 2: Start the Docusaurus Frontend

In a new terminal, navigate to the project root and start Docusaurus:

```bash
npm start
```

The website should open at `http://localhost:3000` with the chatbot icon in the bottom-right corner.

## How It Works

1. **User Input**: User types a question and clicks send
2. **API Call**: The chatbot sends a POST request to `http://localhost:8000/chat` with the query
3. **RAG Processing**: The backend uses the RAG agent to search the textbook content and generate a response
4. **Display Response**: The AI response is displayed in the chat interface

## API Integration

The chatbot communicates with the RAG bot using the following endpoint:

**Endpoint**: `POST http://localhost:8000/chat`

**Request Body**:
```json
{
  "query": "What is ROS 2?"
}
```

**Response**:
```json
{
  "answer": "ROS 2 is the second generation of the Robot Operating System..."
}
```

## Error Handling

The chatbot includes comprehensive error handling:

- **Connection Errors**: If the RAG bot is not running, users see a helpful error message
- **Rate Limiting**: Handles 429 errors gracefully
- **Network Issues**: Displays user-friendly error messages

## Customization

### Changing the API URL

If your RAG bot runs on a different port or host, update the API URL in `index.tsx`:

```tsx
const response = await fetch('http://localhost:8000/chat', {
  // Change to your API URL
});
```

### Styling

All styles are in `styles.module.css`. The chatbot uses:
- CSS modules for scoped styling
- Responsive breakpoints for mobile support
- Dark mode support via Docusaurus theme

## Troubleshooting

### Chatbot shows "Server not running" error

**Solution**: Make sure the RAG bot backend is running on port 8000:
```bash
cd rag-bot
python main.py
```

### CORS errors in browser console

**Solution**: The RAG bot's `main.py` already includes CORS middleware for `localhost:3000`. If you're running on a different port, update the `origins` list in `main.py`:

```python
origins = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
    # Add your custom origin here
]
```

### Chatbot is outside the viewport

**Solution**: The chatbot now uses responsive sizing with `min()` and `max-width`/`max-height` to ensure it stays within viewport bounds on all screen sizes.

## Development

To modify the chatbot behavior:

1. **UI Changes**: Edit `index.tsx` for component logic
2. **Styling**: Edit `styles.module.css` for visual design
3. **API Integration**: Modify the `handleSendMessage` function in `index.tsx`

## Testing

Test the chatbot with sample queries:

- "What is ROS 2?"
- "How do I simulate a humanoid robot?"
- "Explain AI-powered navigation"
- "What is URDF?"

The RAG bot should provide relevant answers based on the textbook content stored in the Qdrant vector database.

## File Structure

```
src/
├── components/
│   └── Chatbot/
│       ├── index.tsx          # Main chatbot component with RAG integration
│       ├── styles.module.css  # Responsive styles
│       └── README.md          # This file
└── theme/
    └── Root.tsx               # Theme wrapper that includes chatbot on all pages
```

## Technologies Used

- **React 19**: Component framework
- **TypeScript**: Type safety
- **CSS Modules**: Scoped styling
- **Docusaurus 3.9**: Documentation framework
- **FastAPI**: RAG bot backend
- **Fetch API**: HTTP requests to backend
