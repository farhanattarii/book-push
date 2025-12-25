---
sidebar_position: 3
---

# Cognitive Planning using LLMs for ROS 2

## Introduction

This chapter explores how to use Large Language Models (LLMs) for cognitive planning in robotics. Students will learn how to convert natural language commands into sequences of ROS 2 actions, with safety validation and complex task execution.

## Understanding Cognitive Planning

Cognitive planning in robotics refers to the process of taking high-level commands and breaking them down into executable actions. With LLMs, we can interpret natural language and generate appropriate robotic behaviors.

## Using LLMs for Natural Language Processing

LLMs excel at understanding context and generating structured outputs. We'll use OpenAI's GPT models to convert natural language commands into structured action sequences.

### Basic Command Processing

```python
import openai
import json
import os

class CognitivePlanner:
    def __init__(self):
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    def plan_actions(self, command):
        """
        Convert a natural language command into a sequence of ROS 2 actions.
        """
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

            # Extract the JSON from the response
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

# Example usage
if __name__ == "__main__":
    planner = CognitivePlanner()
    result = planner.plan_actions("Move to the kitchen and pick up the red cup")
    print(json.dumps(result, indent=2))
```

## Advanced Command Processing

For more complex commands, we need to consider context and dependencies between actions:

```python
class AdvancedCognitivePlanner(CognitivePlanner):
    def plan_complex_command(self, command, context=None):
        """
        Plan actions for complex commands with context awareness.
        """
        context_str = f"Context: {context}" if context else "No additional context provided."

        prompt = f"""
        {context_str}

        Convert the following complex natural language command into a sequence of ROS 2 actions.
        Consider dependencies, safety constraints, and optimal execution order.
        Return the result as a JSON object with 'action_sequence' and 'reasoning'.

        Command: "{command}"

        Example output format:
        {{
          "action_sequence": [
            {{
              "action_type": "object_detection",
              "parameters": {{
                "target": "red cup"
              }},
              "id": "detect_cup"
            }},
            {{
              "action_type": "move_to_pose",
              "parameters": {{
                "x": 1.0,
                "y": 2.0,
                "theta": 0.0
              }},
              "depends_on": ["detect_cup"],
              "id": "move_to_cup"
            }}
          ],
          "reasoning": "Explanation of why these actions were chosen"
        }}

        Actions should be valid ROS 2 action types.
        Include dependencies where one action must complete before another starts.
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()

            # Remove any markdown formatting if present
            if content.startswith("```json"):
                content = content[7:]  # Remove ```json
            if content.endswith("```"):
                content = content[:-3]  # Remove ```

            result = json.loads(content)
            result["success"] = True
            return result
        except Exception as e:
            return {
                "success": False,
                "error": f"Complex planning failed: {str(e)}",
                "action_sequence": [],
                "reasoning": ""
            }
```

## Safety Validation

Before executing any action sequence, we must validate it against safety constraints:

```python
class SafetyValidator:
    def __init__(self):
        # Define safety boundaries
        self.safety_boundaries = {
            "position": {
                "min_x": -10.0, "max_x": 10.0,
                "min_y": -10.0, "max_y": 10.0,
                "min_z": 0.0, "max_z": 2.0
            },
            "speed": {
                "max_linear": 1.0,  # m/s
                "max_angular": 1.0  # rad/s
            },
            "force": {
                "max_gripper": 50.0  # Newtons
            }
        }

    def validate_action_sequence(self, action_sequence):
        """
        Validate an action sequence against safety constraints.
        """
        validation_results = []

        for i, action in enumerate(action_sequence):
            action_type = action.get("action_type", "")
            parameters = action.get("parameters", {})

            result = {
                "action_index": i,
                "action_type": action_type,
                "valid": True,
                "violations": []
            }

            # Validate based on action type
            if action_type == "move_to_pose":
                x = parameters.get("x", 0)
                y = parameters.get("y", 0)

                if (x < self.safety_boundaries["position"]["min_x"] or
                    x > self.safety_boundaries["position"]["max_x"] or
                    y < self.safety_boundaries["position"]["min_y"] or
                    y > self.safety_boundaries["position"]["max_y"]):
                    result["valid"] = False
                    result["violations"].append(f"Position ({x}, {y}) is outside safe boundaries")

            elif action_type == "gripper_control":
                force = parameters.get("force", 0)
                if force > self.safety_boundaries["force"]["max_gripper"]:
                    result["valid"] = False
                    result["violations"].append(f"Gripper force {force}N exceeds maximum {self.safety_boundaries['force']['max_gripper']}N")

            validation_results.append(result)

        all_valid = all(result["valid"] for result in validation_results)

        return {
            "overall_valid": all_valid,
            "validation_results": validation_results,
            "action_sequence": action_sequence
        }
```

## Integration Example

Here's how to integrate cognitive planning with safety validation:

```python
class VLACognitivePlanner:
    def __init__(self):
        self.planner = AdvancedCognitivePlanner()
        self.safety_validator = SafetyValidator()

    def process_command(self, command, context=None):
        """
        Complete pipeline: command -> plan -> validate -> return safe action sequence
        """
        # Step 1: Generate action plan
        plan_result = self.planner.plan_complex_command(command, context)

        if not plan_result["success"]:
            return plan_result

        # Step 2: Validate the plan against safety constraints
        validation_result = self.safety_validator.validate_action_sequence(
            plan_result["action_sequence"]
        )

        # Step 3: Return the complete result
        return {
            "success": True,
            "action_sequence": validation_result["action_sequence"],
            "validation": validation_result,
            "reasoning": plan_result["reasoning"],
            "command": command
        }

# Example usage
if __name__ == "__main__":
    vla_planner = VLACognitivePlanner()
    result = vla_planner.process_command(
        "Navigate to position (5, 3) and pick up the blue box",
        context="Robot is currently at (0, 0)"
    )
    print(json.dumps(result, indent=2))
```

## Handling Complex Multi-Step Tasks

For complex tasks that require multiple phases, we can implement hierarchical planning:

```python
class HierarchicalPlanner:
    def __init__(self):
        self.low_level_planner = VLACognitivePlanner()

    def plan_hierarchical_task(self, high_level_command):
        """
        Break down a high-level command into phases and plan each phase.
        """
        # First, ask the LLM to break down the high-level command
        breakdown_prompt = f"""
        Break down the following high-level command into 2-4 phases of execution.
        Each phase should be a coherent sub-task that can be planned separately.

        Command: "{high_level_command}"

        Return as JSON:
        {{
          "phases": [
            {{"description": "Phase 1 description", "sub_command": "Specific command for this phase"}},
            {{"description": "Phase 2 description", "sub_command": "Specific command for this phase"}}
          ]
        }}
        """

        try:
            response = self.low_level_planner.planner.client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": breakdown_prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()
            if content.startswith("```json"):
                content = content[7:]
            if content.endswith("```"):
                content = content[:-3]

            phase_breakdown = json.loads(content)

            # Plan each phase separately
            full_plan = {
                "high_level_command": high_level_command,
                "phases": [],
                "overall_success": True
            }

            for i, phase in enumerate(phase_breakdown["phases"]):
                phase_result = self.low_level_planner.process_command(phase["sub_command"])

                full_plan["phases"].append({
                    "phase_number": i + 1,
                    "description": phase["description"],
                    "sub_command": phase["sub_command"],
                    "plan": phase_result
                })

                if not phase_result["success"]:
                    full_plan["overall_success"] = False

            return full_plan
        except Exception as e:
            return {
                "success": False,
                "error": f"Hierarchical planning failed: {str(e)}"
            }
```

## Summary

In this chapter, we've explored how to use LLMs for cognitive planning in robotics. We've implemented:
- Basic command-to-action conversion using GPT models
- Advanced planning with context awareness and dependencies
- Safety validation to ensure robot actions are within safe boundaries
- Hierarchical planning for complex multi-step tasks

The next chapter will integrate these components into a complete capstone project that demonstrates the full VLA pipeline.

## References

- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [GPT Best Practices](https://platform.openai.com/docs/guides/gpt-best-practices)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-ROS2-Actions-In-Different-Languages.html)
- [Large Language Models in Robotics](https://arxiv.org/abs/2304.06126)