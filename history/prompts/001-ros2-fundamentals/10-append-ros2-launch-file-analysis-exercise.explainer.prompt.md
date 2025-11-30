---
id: 10
title: append ros2 launch file analysis exercise to lesson
stage: explainer
date: 2025-11-30
surface: agent
model: gemini-2.5-flash
feature: 001-ros2-fundamentals
branch: 001-ros2-fundamentals
user: user
command: Develop a conceptual exercise where learners analyze a launch file for previously developed publisher/subscriber nodes in docs/01-ros2-fundamentals/lesson6.md. The exercise should refer to the my_robot.launch.py file created in the previous step and ask questions about its structure, what nodes it launches, and how parameters (if any) are passed. Provide the content for the exercise to be appended to lesson6.md.
labels: ["ros2", "launch_files", "conceptual_exercise"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - C:\Users\dell\Desktop\physical-ai\worktrees\chapter1\docs\01-ros2-fundamentals\lesson6.md
tests:
null
---

## Prompt

Develop a conceptual exercise where learners analyze a launch file for previously developed publisher/subscriber nodes in docs/01-ros2-fundamentals/lesson6.md. The exercise should refer to the my_robot.launch.py file created in the previous step and ask questions about its structure, what nodes it launches, and how parameters (if any) are passed. Provide the content for the exercise to be appended to lesson6.md.

## Response snapshot

## Practice

You have previously developed a publisher/subscriber node and created a `my_robot.launch.py` file to launch them. Let's analyze its structure and functionality.

Consider the following `my_robot.launch.py` file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher',
            name='robot_publisher',
            parameters=[{'publish_frequency': 10.0}]
        ),
        Node(
            package='my_package',
            executable='my_subscriber',
            name='robot_subscriber'
        ),
    ])
```

Output:
This launch file starts a `my_publisher` node and a `my_subscriber` node from `my_package`. The `robot_publisher` node's `publish_frequency` is set to 10.0 Hz.

Now, answer the following questions about this launch file:

1.  **Launch File Purpose**: What is the primary purpose of `my_robot.launch.py`?
2.  **Nodes Launched**: Which two ROS 2 nodes are launched by this file?
3.  **Package Names**: From which package do these nodes originate?
4.  **Node Renaming**: Are any nodes assigned a custom name different from their executable name? If so, which one(s)?
5.  **Parameter Passing**: Is a parameter being passed to any of the nodes? If yes, which node receives it, what is the parameter name, and what is its value?
6.  **Extensibility**: If you wanted to add a third node, `sensor_reader`, from `another_package` to this launch file, where would you add it and what essential information would you need to provide?

> **💬 AI Colearning Prompt**: Reflect on your answers. Discuss with the AI the benefits of using launch files for managing complex robotic systems. How do they simplify development and deployment?

## Outcome

- ✅ Impact: Appended a conceptual exercise to `lesson6.md` for analyzing ROS 2 launch files.
- 🧪 Tests: null
- 📁 Files: Updated C:\Users\dell\Desktop\physical-ai\worktrees\chapter1\docs\01-ros2-fundamentals\lesson6.md
- 🔁 Next prompts: The user might ask to review or further expand the lesson content.
- 🧠 Reflection: The exercise provides practical application of launch file concepts, enhancing learning.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
