import google.generativeai as genai
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure the API key for Gemini
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("GEMINI_API_KEY environment variable not set")

genai.configure(api_key=api_key)

# Initialize the model (you can change the model name if needed)
model = genai.GenerativeModel('gemini-pro')

def get_gemini_response(user_message):
    """
    Function to get response from the Gemini model
    """
    try:
        # Generate content based on the user message
        response = model.generate_content(user_message)
        return response.text
    except Exception as e:
        # Handle any errors that occur during generation
        return f"Error generating response: {str(e)}"