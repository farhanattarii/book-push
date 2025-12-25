---
sidebar_position: 2
---

# Voice-to-Action with OpenAI Whisper

## Introduction

This chapter covers the implementation of voice-to-action conversion using OpenAI Whisper for speech recognition. Students will learn how to capture voice commands, convert them to text, and process them for robotic action execution.

## Setting up OpenAI Whisper

OpenAI Whisper is a state-of-the-art speech recognition system that can convert audio to text with high accuracy. To use Whisper in our VLA system, we need to integrate with the OpenAI API.

### Installation

First, install the required dependencies:

```bash
pip install openai
```

### Basic Voice Processing

Here's a simple example of how to use OpenAI Whisper to convert audio to text:

```python
import openai
import os

# Load your API key from environment variables
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio(audio_file_path):
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe("whisper-1", audio_file)
    return transcript.text

# Example usage
# result = transcribe_audio("path/to/audio.wav")
# print(result)
```

## Processing Audio Input

The VLA system needs to handle real-time audio input from users. We'll implement an audio capture mechanism that can convert microphone input to text.

```python
import pyaudio
import wave
import tempfile
import os

def record_audio(duration=5, filename="temp_audio.wav"):
    chunk = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100

    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=chunk)

    print("Recording...")
    frames = []

    for i in range(0, int(RATE / chunk * duration)):
        data = stream.read(chunk)
        frames.append(data)

    print("Finished recording.")

    stream.stop_stream()
    stream.close()
    p.terminate()

    # Save the recorded audio to a temporary file
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    return filename
```

## Handling Speech Recognition Errors

Sometimes, Whisper might not be able to accurately transcribe audio. Our system should handle these cases gracefully:

```python
def handle_speech_recognition_errors(audio_file_path):
    try:
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.Audio.transcribe("whisper-1", audio_file)

        # Check confidence or quality metrics if available
        if len(transcript.text.strip()) == 0:
            return {"success": False, "error": "No speech detected"}

        return {"success": True, "text": transcript.text}
    except Exception as e:
        return {"success": False, "error": f"Transcription failed: {str(e)}"}
```

## Practical Examples

Let's implement a complete example that captures audio from the microphone and transcribes it using Whisper:

```python
import openai
import pyaudio
import wave
import tempfile
import os

class VoiceToAction:
    def __init__(self):
        openai.api_key = os.getenv("OPENAI_API_KEY")

    def record_and_transcribe(self, duration=5):
        # Record audio
        temp_filename = "temp_recording.wav"
        self.record_audio(duration, temp_filename)

        # Transcribe audio
        try:
            with open(temp_filename, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)

            # Clean up temporary file
            os.remove(temp_filename)

            return transcript.text
        except Exception as e:
            os.remove(temp_filename)
            raise e

    def record_audio(self, duration, filename):
        chunk = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100

        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=chunk)

        print("Recording...")
        frames = []

        for i in range(0, int(RATE / chunk * duration)):
            data = stream.read(chunk)
            frames.append(data)

        print("Finished recording.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

# Example usage
if __name__ == "__main__":
    vta = VoiceToAction()
    result = vta.record_and_transcribe(duration=5)
    print(f"Transcribed: {result}")
```

## Summary

In this chapter, we've covered the basics of using OpenAI Whisper for voice-to-text conversion. We've implemented audio capture functionality and error handling to create a robust voice processing system. The next chapter will cover how to use LLMs for cognitive planning to convert these text commands into ROS 2 actions.

## References

- [OpenAI Whisper API Documentation](https://platform.openai.com/docs/guides/speech-to-text)
- [OpenAI Python Library](https://github.com/openai/openai-python)
- [PyAudio Documentation](https://pyaudio.readthedocs.io/)
- [ROS 2 Audio Processing Tutorials](https://docs.ros.org/en/humble/Tutorials.html#audio-processing)