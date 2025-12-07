export async function postChatQuery(query: string, context: string): Promise<string> {
  try {
    const response = await fetch('/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ query, context }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.detail || 'Failed to fetch chatbot response');
    }

    const data = await response.json();
    return data.response;
  } catch (error) {
    console.error('Error in postChatQuery:', error);
    throw error;
  }
}
