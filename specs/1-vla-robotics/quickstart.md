# Quickstart: VLA Robotics Module

## Prerequisites

Before starting with the Vision-Language-Action (VLA) module, ensure you have the following installed:

- Python 3.11 or higher
- Node.js 18 or higher
- ROS 2 Humble Hawksbill (for robot simulation)
- OpenAI API key
- Git

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up the Docusaurus Documentation
```bash
cd book_frontend
npm install
```

### 3. Install Python Dependencies
```bash
pip install openai openai-whisper rospy ros2-interfaces
```

### 4. Set Environment Variables
Create a `.env` file in the project root:
```env
OPENAI_API_KEY=your_openai_api_key_here
ROS_DOMAIN_ID=42
```

## Running the Examples

### 1. Start the Documentation Server
```bash
cd book_frontend
npm start
```

### 2. Run the Voice-to-Action Example
```bash
cd src/examples
python vla_demo.py
```

### 3. Test with Sample Commands
Speak or provide text commands such as:
- "Move the robot forward 1 meter"
- "Turn the robot 90 degrees to the right"
- "Pick up the object in front of you"

## Docusaurus Chapters Structure

The VLA module consists of 3 main chapters:

### Chapter 1: Voice-to-Action with OpenAI Whisper
Location: `book_frontend/docs/vla/voice-to-action-whisper.md`

This chapter covers:
- Setting up OpenAI Whisper for speech recognition
- Processing audio input and converting to text
- Handling speech recognition errors
- Practical examples with code

### Chapter 2: Cognitive Planning using LLMs for ROS 2
Location: `book_frontend/docs/vla/cognitive-planning-llms.md`

This chapter covers:
- Using LLMs to interpret natural language commands
- Converting commands to ROS 2 action sequences
- Safety validation of generated actions
- Planning complex multi-step tasks

### Chapter 3: Capstone: Autonomous Humanoid executing tasks
Location: `book_frontend/docs/vla/capstone-autonomous-humanoid.md`

This chapter covers:
- Complete integration of voice, planning, and execution
- Running the full VLA pipeline with simulated robot
- Advanced examples and troubleshooting
- Extending the system for different robot platforms

## Running the ROS 2 Simulation

### 1. Start Gazebo Simulation
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run the VLA Node
```bash
source /opt/ros/humble/setup.bash
cd src/vla
python ros2_action_generator.py
```

### 3. Issue Voice Commands
Use the provided command-line interface to issue voice commands to the simulated robot.

## Troubleshooting

### Common Issues:

1. **"ModuleNotFoundError: No module named 'openai'"**
   - Run: `pip install openai`

2. **"ROS_DISTRO not set"**
   - Run: `source /opt/ros/humble/setup.bash`

3. **"OpenAI API key not found"**
   - Ensure your API key is set in the `.env` file

4. **"Whisper model not found"**
   - Run: `python -c "import whisper; whisper.load_model('base')"` to download the model

## Next Steps

After completing the quickstart:
1. Read through Chapter 1 to understand voice processing
2. Work through the examples in Chapter 2 for cognitive planning
3. Complete the capstone project in Chapter 3
4. Experiment with custom voice commands and robot actions