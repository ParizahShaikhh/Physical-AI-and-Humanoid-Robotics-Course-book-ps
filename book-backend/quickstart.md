# Quickstart Guide for RAG Agent

## Prerequisites

- Python 3.8+
- OpenAI API key
- Qdrant Cloud access (URL and API key)
- Existing book content indexed in Qdrant
- Spec-2 retrieval pipeline available

## Environment Setup

1. Install required packages:
```bash
pip install openai python-dotenv qdrant-client
```

2. Create `.env` file with the following variables:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key  # if needed for retrieval
```

## Running the Agent

1. Start the agent:
```bash
python agent.py
```

## Basic Usage

1. Initialize the agent:
```python
from agent import RAGAgent
agent = RAGAgent()
```

2. Ask questions:
```python
response = agent.ask("Explain ROS 2 concepts")
print(response)
```

## Query Modes

### Full-book mode (default):
```python
response = agent.ask("What is the main concept of physical AI?", mode="full-book")
```

### Selected-text mode:
```python
response = agent.ask("How does NVIDIA Isaac work?",
                     mode="selected-text",
                     filters={"source_url": "specific-url-here"})
```

## Example Questions

- "Explain the difference between ROS 1 and ROS 2"
- "What are the key concepts in Chapter 3?"
- "How does simulation-to-reality transfer work?"
- "Describe the architecture of humanoid robotics systems"