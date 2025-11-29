<!--
Sync Impact Report:
Version change: None → 1.0.0
List of modified principles:
  - PRINCIPLE_1_NAME: "Accuracy and Technical Rigor"
  - PRINCIPLE_2_NAME: "Clarity and Accessibility"
  - PRINCIPLE_3_NAME: "Practical Application and Embodied Intelligence"
  - PRINCIPLE_4_NAME: "Ethical Considerations"
  - PRINCIPLE_5_NAME: "Content Structure and Flow"
  - PRINCIPLE_6_NAME: "Technical Stack and Tools"
Added sections: Project Vision and Goals, Content Guidelines, Hardware Context
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Project Vision and Goals
The primary vision of this project is to create a comprehensive textbook for teaching a course in Physical AI & Humanoid Robotics. This book aims to bridge the gap between digital AI and the physical world, enabling students to apply their AI knowledge to control humanoid robots in simulated and real-world environments. The textbook will serve as a foundational resource for a capstone quarter introducing Physical AI systems that function in reality and comprehend physical laws.

## Core Principles

### I. Accuracy and Technical Rigor
All content must be factually correct, up-to-date, and provide robust technical explanations for all covered technologies (ROS 2, Gazebo, Unity, NVIDIA Isaac, GPT models).

### II. Clarity and Accessibility
The textbook will be written for students with a foundational understanding of AI, balancing technical depth with pedagogical clarity. Concepts should be explained in an understandable manner, with clear examples.

### III. Practical Application and Embodied Intelligence
Emphasize hands-on learning, practical application, and the principles of embodied intelligence. The content should focus on designing, simulating, and deploying humanoid robots for natural human interactions.

### IV. Ethical Considerations
Integrate discussions on the ethical implications of AI and humanoid robotics, promoting responsible development and deployment.

### V. Content Structure and Flow
The textbook will follow a logical progression, starting from foundational concepts (Physical AI, ROS 2) and advancing to more complex topics (simulation, NVIDIA Isaac, humanoid design, conversational AI). Each section and chapter will contribute to achieving the defined learning outcomes.

### VI. Technical Stack and Tools
The textbook will extensively cover and demonstrate the use of ROS 2, Gazebo, Unity, NVIDIA Isaac SDK/Sim/ROS, and GPT models, with Python as the primary programming language. URDF will be used for robot modeling.

## Content Guidelines
*   **Engaging Style:** Content must be fun, relatable, and use easy English with appropriate emojis to enhance student engagement.
*   **Conciseness:** Paragraphs must be a maximum of 100 characters and list item explanations must be a maximum of 50 characters.
*   **Code Examples:** Provide clear, concise, and functional code examples, primarily in Python, for ROS 2, NVIDIA Isaac, and GPT integrations. Code snippets will adhere to best practices for readability and maintainability.
*   **Diagrams and Visualizations:** Utilize diagrams, illustrations, and visual aids to explain complex robotic architectures, algorithms, and simulation environments.
*   **Terminology:** Maintain consistent terminology, formatting, and presentation style throughout the book.
*   **Course Alignment:** Content will be directly aligned with the specified course modules and learning outcomes.

### VII. Engaging and Concise Communication
Content must be fun, relatable, and use easy English with appropriate emojis. Paragraphs and list item explanations must be a maximum of 100 characters to ensure conciseness and student engagement.

## Hardware Context
The textbook acknowledges and will guide students through the demanding hardware requirements for Physical AI, including:
*   High-Performance Workstations with NVIDIA RTX GPUs (e.g., RTX 4070 Ti or higher) for physics simulation, visual perception, and generative AI.
*   Edge Computing Kits (e.g., NVIDIA Jetson Orin Nano/NX) for physical AI deployment and understanding resource constraints.
*   Sensors such as Intel RealSense D435i/D455 (RGB-D cameras) and USB IMUs.
*   Voice Interfaces like ReSpeaker USB Mic Array.
*   Discussion of various robot lab options (Quadrupeds like Unitree Go2 Edu, Miniature Humanoids like Unitree G1/Robotis OP3, or "Premium" labs with Unitree G1 Humanoid).
*   Consideration of cloud-native lab solutions (e.g., AWS RoboMaker, NVIDIA Omniverse Cloud) as alternatives to on-premise labs.

## Governance
All PRs/reviews must verify compliance; Complexity must be justified; Use this constitution for runtime development guidance.

**Version**: 1.0.1 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
