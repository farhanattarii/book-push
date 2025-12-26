---
sidebar_position: 5
---

# Capstone: Autonomous Humanoid executing tasks

## Introduction

This capstone chapter brings together all the components we've developed in previous chapters to create a complete Vision-Language-Action (VLA) system. Students will learn how to integrate voice processing, cognitive planning, and ROS 2 action execution to create an autonomous humanoid robot that responds to voice commands.

## Complete VLA Pipeline Architecture

The complete VLA system consists of three main components working together:

1. **Voice Processing Layer**: Converts speech to text using OpenAI Whisper
2. **Cognitive Planning Layer**: Uses LLMs to generate action sequences from text commands
3. **Execution Layer**: Executes ROS 2 actions on the humanoid robot with safety validation

## Complete System Implementation

Here's the complete implementation that ties together all components:

```python
import openai
import json
import os
import asyncio
from typing import Dict, List, Any

class VoiceCommand:
    """Represents a voice command with its processing state."""
    def __init__(self, audio_data=None, text_transcription="", confidence=0.0, user_id=""):
        self.id = os.urandom(8).hex()
        self.audio_data = audio_data
        self.text_transcription = text_transcription
        self.confidence = confidence
        self.user_id = user_id
        self.timestamp = None
        self.status = "created"

class CognitivePlan:
    """Represents a planned sequence of actions."""
    def __init__(self, voice_command_id, action_sequence, reasoning, status="pending"):
        self.id = os.urandom(8).hex()
        self.voice_command_id = voice_command_id
        self.action_sequence = action_sequence
        self.reasoning = reasoning
        self.status = status
        self.generated_at = None

class ROS2Action:
    """Represents a single ROS 2 action."""
    def __init__(self, action_type, parameters, priority=5):
        self.id = os.urandom(8).hex()
        self.action_type = action_type
        self.parameters = parameters
        self.priority = priority
        self.created_at = None
        self.safety_validation = None

class SafetyValidator:
    """Validates actions against safety constraints."""
    def __init__(self):
        self.safety_boundaries = {
            "position": {
                "min_x": -10.0, "max_x": 10.0,
                "min_y": -10.0, "max_y": 10.0,
                "min_z": 0.0, "max_z": 2.0
            },
            "speed": {
                "max_linear": 1.0,
                "max_angular": 1.0
            },
            "force": {
                "max_gripper": 50.0
            }
        }

    def validate_action(self, action: ROS2Action) -> Dict[str, Any]:
        """Validate a single action against safety constraints."""
        result = {
            "action_id": action.id,
            "valid": True,
            "violations": []
        }

        if action.action_type == "move_to_pose":
            x = action.parameters.get("x", 0)
            y = action.parameters.get("y", 0)

            if (x < self.safety_boundaries["position"]["min_x"] or
                x > self.safety_boundaries["position"]["max_x"] or
                y < self.safety_boundaries["position"]["min_y"] or
                y > self.safety_boundaries["position"]["max_y"]):
                result["valid"] = False
                result["violations"].append(f"Position ({x}, {y}) is outside safe boundaries")

        elif action.action_type == "gripper_control":
            force = action.parameters.get("force", 0)
            if force > self.safety_boundaries["force"]["max_gripper"]:
                result["valid"] = False
                result["violations"].append(f"Gripper force {force}N exceeds maximum {self.safety_boundaries['force']['max_gripper']}N")

        return result

class WhisperIntegration:
    """Handles voice-to-text conversion using OpenAI Whisper."""
    def __init__(self):
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def transcribe_audio(self, audio_file_path: str) -> Dict[str, Any]:
        """Transcribe audio file to text."""
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file
                )

            return {
                "success": True,
                "text": transcript.text,
                "confidence": 0.9  # Whisper doesn't provide confidence, using a default
            }
        except Exception as e:
            return {
                "success": False,
                "error": f"Transcription failed: {str(e)}"
            }

class LLMPrompter:
    """Handles cognitive planning using LLMs."""
    def __init__(self):
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def generate_action_sequence(self, command: str) -> Dict[str, Any]:
        """Generate a sequence of ROS 2 actions from a natural language command."""
        prompt = f"""
        Convert the following natural language command into a sequence of ROS 2 actions.
        Return the result as a JSON array of action objects with 'action_type' and 'parameters'.

        Command: "{command}"

        Example output format:
        [
          {{
            "action_type": "move_to_pose",
            "parameters": {{
              "x": 1.0,
              "y": 2.0,
              "theta": 0.0
            }}
          }},
          {{
            "action_type": "gripper_control",
            "parameters": {{
              "action": "open"
            }}
          }}
        ]

        Actions should be valid ROS 2 action types like:
        - move_to_pose
        - move_to_waypoints
        - gripper_control
        - arm_control
        - navigation
        - object_detection

        Only return the JSON array, nothing else.
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()

            # Remove any markdown formatting if present
            if content.startswith("```json"):
                content = content[7:]  # Remove ```json
            if content.endswith("```"):
                content = content[:-3]  # Remove ```

            action_sequence = json.loads(content)
            return {
                "success": True,
                "action_sequence": action_sequence,
                "reasoning": f"Planned actions for command: {command}"
            }
        except Exception as e:
            return {
                "success": False,
                "error": f"Planning failed: {str(e)}",
                "action_sequence": []
            }

class RobotExecutor:
    """Simulates execution of ROS 2 actions on a robot."""
    def __init__(self):
        self.robot_position = {"x": 0.0, "y": 0.0, "theta": 0.0}

    def execute_action(self, action: ROS2Action) -> Dict[str, Any]:
        """Execute a single ROS 2 action."""
        print(f"Executing action: {action.action_type} with parameters: {action.parameters}")

        # Simulate action execution based on type
        if action.action_type == "move_to_pose":
            new_x = action.parameters.get("x", self.robot_position["x"])
            new_y = action.parameters.get("y", self.robot_position["y"])
            new_theta = action.parameters.get("theta", self.robot_position["theta"])

            # Update robot position
            self.robot_position = {"x": new_x, "y": new_y, "theta": new_theta}
            print(f"Robot moved to position: {self.robot_position}")

        elif action.action_type == "gripper_control":
            action_param = action.parameters.get("action", "unknown")
            force = action.parameters.get("force", 10.0)
            print(f"Gripper {action_param} with force {force}N")

        elif action.action_type == "arm_control":
            joint_positions = action.parameters.get("joint_positions", [])
            print(f"Arm moved to joint positions: {joint_positions}")

        # Simulate execution time
        import time
        time.sleep(0.5)  # Simulate action execution time

        return {
            "success": True,
            "action_id": action.id,
            "result": f"Successfully executed {action.action_type}",
            "new_robot_state": self.robot_position
        }

class VLAPipeline:
    """Complete VLA pipeline combining all components."""
    def __init__(self):
        self.whisper = WhisperIntegration()
        self.llm_planner = LLMPrompter()
        self.safety_validator = SafetyValidator()
        self.robot_executor = RobotExecutor()

    def process_voice_command(self, audio_file_path: str) -> Dict[str, Any]:
        """Complete pipeline: audio -> text -> plan -> validate -> execute."""
        # Step 1: Transcribe audio to text
        transcription_result = self.whisper.transcribe_audio(audio_file_path)

        if not transcription_result["success"]:
            return {
                "success": False,
                "error": f"Audio transcription failed: {transcription_result['error']}"
            }

        text_command = transcription_result["text"]
        print(f"Transcribed command: {text_command}")

        # Step 2: Generate action sequence from text
        planning_result = self.llm_planner.generate_action_sequence(text_command)

        if not planning_result["success"]:
            return {
                "success": False,
                "error": f"Action planning failed: {planning_result['error']}"
            }

        action_sequence = planning_result["action_sequence"]
        print(f"Generated action sequence: {len(action_sequence)} actions")

        # Step 3: Validate each action against safety constraints
        validated_actions = []
        for action_data in action_sequence:
            action = ROS2Action(
                action_type=action_data["action_type"],
                parameters=action_data["parameters"]
            )

            validation_result = self.safety_validator.validate_action(action)

            if not validation_result["valid"]:
                return {
                    "success": False,
                    "error": f"Safety validation failed: {'; '.join(validation_result['violations'])}"
                }

            validated_actions.append(action)

        print(f"All actions passed safety validation")

        # Step 4: Execute the validated actions
        execution_results = []
        for action in validated_actions:
            execution_result = self.robot_executor.execute_action(action)
            execution_results.append(execution_result)

            if not execution_result["success"]:
                return {
                    "success": False,
                    "error": f"Action execution failed: {execution_result['result']}"
                }

        return {
            "success": True,
            "original_command": text_command,
            "action_sequence": action_sequence,
            "execution_results": execution_results,
            "final_robot_state": self.robot_executor.robot_position
        }

# Example usage of the complete pipeline
if __name__ == "__main__":
    vla_pipeline = VLAPipeline()

    # This would be a real audio file path in practice
    # For demonstration, we'll simulate the transcription step
    print("VLA Pipeline initialized. Ready to process voice commands.")
    print("Example usage would be:")
    print("# result = vla_pipeline.process_voice_command('path/to/audio.wav')")
    print("# print(result)")
```

## Running the Complete System

To run the complete system with a simulated audio file, you would:

```python
def run_demo():
    """Run a demonstration of the complete VLA system."""
    vla_pipeline = VLAPipeline()

    # Example commands to test
    test_commands = [
        "Move forward by 1 meter",
        "Turn 90 degrees to the right",
        "Pick up the object in front of you"
    ]

    print("Starting VLA System Demo...")
    print("=" * 50)

    for i, command in enumerate(test_commands, 1):
        print(f"\nTest {i}: {command}")
        print("-" * 30)

        # Since we don't have actual audio files, we'll simulate by directly using the LLM planner
        # In a real system, this would be the result of audio transcription
        planning_result = vla_pipeline.llm_planner.generate_action_sequence(command)

        if not planning_result["success"]:
            print(f"Planning failed: {planning_result['error']}")
            continue

        action_sequence = planning_result["action_sequence"]
        print(f"Planned actions: {len(action_sequence)}")

        # Validate and execute
        validated_actions = []
        for action_data in action_sequence:
            action = ROS2Action(
                action_type=action_data["action_type"],
                parameters=action_data["parameters"]
            )

            validation_result = vla_pipeline.safety_validator.validate_action(action)

            if not validation_result["valid"]:
                print(f"Safety validation failed: {'; '.join(validation_result['violations'])}")
                continue

            validated_actions.append(action)

        print(f"Validated {len(validated_actions)} actions")

        # Execute the validated actions
        for action in validated_actions:
            execution_result = vla_pipeline.robot_executor.execute_action(action)
            if execution_result["success"]:
                print(f"✓ Executed: {action.action_type}")
            else:
                print(f"✗ Failed: {execution_result['result']}")

    print("\n" + "=" * 50)
    print("Demo completed!")
    print(f"Final robot position: {vla_pipeline.robot_executor.robot_position}")

if __name__ == "__main__":
    run_demo()
```

## Troubleshooting Common Issues

### Audio Processing Issues

If audio transcription is failing:

1. Check that your OpenAI API key is properly set in the environment
2. Verify the audio file format is supported (WAV, MP3, etc.)
3. Ensure the audio file is not corrupted

### LLM Planning Issues

If action planning is not working as expected:

1. Verify your OpenAI API key has sufficient quota
2. Try using a more specific command
3. Consider using a more capable model (GPT-4 instead of GPT-3.5)

### Safety Validation Issues

If actions are failing safety validation:

1. Check that the target positions are within the defined safety boundaries
2. Verify that force parameters are within acceptable limits
3. Adjust safety boundaries if they are too restrictive for your use case

## Extending the System

The VLA system can be extended in several ways:

1. **Additional Action Types**: Add support for more ROS 2 action types
2. **Improved Safety**: Add more sophisticated safety checks and monitoring
3. **Learning Capabilities**: Implement learning from successful and failed executions
4. **Multi-Modal Input**: Add vision processing to complement voice commands

## Summary

In this capstone chapter, we've implemented a complete VLA system that:
- Processes voice commands using OpenAI Whisper
- Plans robotic actions using LLMs
- Validates actions against safety constraints
- Executes actions on a simulated robot

This system demonstrates the integration of vision, language, and action components that make up modern AI-robotics systems. Students can extend this foundation to work with real robots and more complex scenarios.

## References

- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [ROS 2 Humble Hawksbill Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Vision-Language-Action Models Research](https://arxiv.org/abs/2209.06588)
- [Humanoid Robotics Development Guide](https://humanoid.ros.org/)