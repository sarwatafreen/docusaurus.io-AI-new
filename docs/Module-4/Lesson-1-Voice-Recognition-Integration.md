---
sidebar_position: 1
---

# Lesson 1: Voice Recognition Integration (Whisper + GPT)

In this lesson, we'll integrate voice recognition capabilities into our robot using OpenAI's Whisper for speech-to-text and GPT for natural language understanding. This will enable your robot to comprehend voice commands like "Robot, bring water".

## Prerequisites
- Python 3.8+ with pip
- OpenAI API key
- Microphone input capabilities (real or simulated)
- Completed Modules 1-3

## What You'll Build
By the end of this lesson, you'll have:
1. Integrated Whisper for speech-to-text conversion
2. Connected GPT for natural language processing
3. Created a voice command processing pipeline
4. Tested voice recognition with your simulated robot

## Setting Up Whisper and GPT

First, install the required packages:

```bash
pip install openai speechrecognition pyaudio transformers torch
```

## Implementing Voice Recognition Pipeline

Create a Python script to handle voice recognition:

```python
import openai
import speech_recognition as sr
import threading
import queue
import time
from transformers import pipeline

# Initialize OpenAI API
openai.api_key = "YOUR_OPENAI_API_KEY"

class VoiceCommandProcessor:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.command_queue = queue.Queue()
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        
        # Initialize Whisper model for local processing (optional, for privacy)
        # self.whisper_model = pipeline("automatic-speech-recognition", model="openai/whisper-tiny")
    
    def listen_for_command(self):
        """Continuously listen for voice commands"""
        print("Listening for commands... Say 'exit' to quit.")
        
        while True:
            try:
                with self.microphone as source:
                    print("Waiting for command...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                
                # Use Whisper for speech recognition (using OpenAI API)
                try:
                    # Save audio to temporary file because Whisper API expects a file
                    with open("temp_audio.wav", "wb") as f:
                        f.write(audio.get_wav_data())
                    
                    with open("temp_audio.wav", "rb") as audio_file:
                        transcript = openai.Audio.transcribe("whisper-1", audio_file)
                    
                    command_text = transcript.text.strip()
                    print(f"Heard: {command_text}")
                    
                    # Process the command
                    if command_text.lower() == "exit":
                        print("Exiting...")
                        break
                    
                    # Send command to GPT for processing
                    self.process_command_with_gpt(command_text)
                    
                except Exception as e:
                    print(f"Error processing audio: {e}")
                    
            except sr.WaitTimeoutError:
                # No command detected, continue listening
                continue
            except KeyboardInterrupt:
                print("Interrupted by user")
                break
    
    def process_command_with_gpt(self, command_text):
        """Process the voice command with GPT to extract intent and parameters"""
        
        # Define the system prompt to guide GPT's responses
        system_prompt = """
        You are a command interpreter for a humanoid robot. 
        Your task is to convert natural language commands into structured robot actions.
        
        Possible actions:
        - 'move_to': Move to a specific location (params: x, y, z coordinates)
        - 'pick_up': Pick up an object (params: object_type, position)
        - 'place_down': Place an object (params: position)
        - 'follow': Follow a person (params: target)
        - 'find': Find an object (params: object_type)
        
        Respond in JSON format with the action type and parameters.
        Example response: {"action": "move_to", "params": {"x": 1.0, "y": 2.0, "z": 0.0}}
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command_text}
                ]
            )
            
            # Extract GPT's response
            gpt_response = response.choices[0].message['content'].strip()
            print(f"GPT processed command: {gpt_response}")
            
            # In a real implementation, you would parse the JSON and send the action to the robot
            # For now, we'll just print the action
            print(f"Robot would execute: {gpt_response}")
            
        except Exception as e:
            print(f"Error processing command with GPT: {e}")

# Main execution
if __name__ == "__main__":
    processor = VoiceCommandProcessor()
    
    # Start the voice command processing
    processor.listen_for_command()
```

## Local Whisper Implementation (Alternative)

For privacy or offline use, you can implement Whisper locally:

```python
import torch
from transformers import WhisperProcessor, WhisperForConditionalGeneration
from datasets import load_dataset

class LocalWhisperProcessor:
    def __init__(self):
        # Load pre-trained Whisper model
        self.processor = WhisperProcessor.from_pretrained("openai/whisper-tiny")
        self.model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-tiny")
        self.model.config.forced_decoder_ids = None
    
    def transcribe(self, audio_file_path):
        """Transcribe audio file to text using local Whisper model"""
        # Load audio data
        dataset = load_dataset("audio", audio_file_path)
        
        # Preprocess the audio
        inputs = self.processor(
            dataset["audio"][0]["array"],
            sampling_rate=16000,
            return_tensors="pt"
        )
        
        # Generate token ids
        predicted_ids = self.model.generate(inputs["input_features"])
        
        # Decode token ids to text
        transcription = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0]
        
        return transcription

# Example usage
# local_whisper = LocalWhisperProcessor()
# transcription = local_whisper.transcribe("path/to/audio/file.wav")
# print(f"Transcribed: {transcription}")
```

## Integrating Voice Commands with ROS 2

To integrate voice commands with your ROS 2 robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Publisher for robot commands
        self.command_publisher = self.create_publisher(String, 'voice_commands', 10)
        
        # Initialize the voice processor
        self.voice_processor = VoiceCommandProcessor()
        
        # Start voice processing in a separate thread
        self.voice_thread = threading.Thread(target=self.voice_processor.listen_for_command)
        self.voice_thread.daemon = True
        self.voice_thread.start()
    
    def publish_command(self, command):
        """Publish voice command to ROS network"""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    voice_command_node = VoiceCommandNode()
    
    try:
        rclpy.spin(voice_command_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quiz

1. What is the purpose of the system prompt in GPT integration?
2. Name two advantages of using local Whisper instead of the API.
3. How would you handle background noise in the voice recognition system?

## Summary

In this lesson, you implemented a voice recognition pipeline using Whisper for speech-to-text and GPT for natural language understanding. You created a system that can listen to commands like "Robot, bring water" and convert them into structured robot actions. In the next lesson, we'll convert these language commands into actual robot actions!