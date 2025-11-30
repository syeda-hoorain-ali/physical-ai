---
id: 7
title: Develop Interactive Quiz for Lesson 1
stage: explainer
date: 2025-11-30
surface: agent
model: opus
feature: 001-ros2-fundamentals
branch: 001-ros2-fundamentals
user: user
command: yes, continue
labels: ["content-generation", "ros2-fundamentals", "lesson1", "quiz"]
links:
  spec: specs/001-ros2-fundamentals/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - docs/01-ros2-fundamentals/lesson1.md
tests:
  null
---

## Prompt

As a lesson-content-writer agent, your task is to develop a simple interactive quiz to check understanding of ROS 2 core concepts. This quiz should be integrated into `docs/01-ros2-fundamentals/lesson1.md`.

Adhere to the following guidelines from the project constitution:
- **Engaging Style:** Content must be fun, relatable, and use easy English with appropriate emojis.
- **Conciseness:** Paragraphs must be a maximum of 100 characters and list item explanations must be a maximum of 50 characters.
- **Technical Rigor:** Provide robust technical explanations.

**Lesson 1 Learning Objectives (from plan.md):**
*   Identify and define the purpose of fundamental ROS 2 concepts: Nodes, Topics, Services, Actions, and Parameters.

**Output format:** Provide the content as a markdown string that can be appended to `docs/01-ros2-fundamentals/lesson1.md`. Do NOT include any additional markdown headers beyond what is needed for this section.

## Response snapshot

## Test Your ROS 2 Knowledge! 🧠

Ready to see how much you've learned about ROS 2? Let's dive into a quick quiz! Choose the best answer for each question. Good luck, future robot master! 🤖

### Quick Quiz Time!

1.  What is the main role of a **Node** in ROS 2?
    *   A. A single process for long-running tasks.
    *   B. A system that stores configuration data.
    *   C. An executable process performing computation.
    *   D. A messaging channel for data exchange.

2.  **Topics** are best described as:
    *   A. Request/response communication channels.
    *   B. Asynchronous data streams for publishers.
    *   C. Synchronous client-server interactions.
    *   D. Persistent configuration values.

3.  How do **Services** facilitate communication?
    *   A. They broadcast data to multiple listeners.
    *   B. They manage long-term goals with feedback.
    *   C. They enable a request-response interaction.
    *   D. They store system settings dynamically.

4.  When would you typically use a ROS 2 **Action**?
    *   A. For simple, instantaneous data broadcasts.
    *   B. To get an immediate response from a server.
    *   C. When executing long-running, cancellable tasks.
    *   D. To adjust a robot's hardware settings.

5.  What is the primary purpose of **Parameters** in ROS 2?
    *   A. To define the message types for topics.
    *   B. To allow nodes to expose configurable values.
    *   C. To handle errors and exceptions in services.
    *   D. To store the history of all node interactions.

---

### Try With AI 🧑‍💻

Think you've got the answers? Share your choices! We can discuss each one and make sure you're mastering these core concepts. What were your answers for each question? Let's check them together! ✨

## Outcome

- ✅ Impact: Interactive quiz added to Lesson 1.
- 🧪 Tests: N/A
- 📁 Files: docs/01-ros2-fundamentals/lesson1.md modified.
- 🔁 Next prompts: Ask user to confirm content, then proceed to T007.
- 🧠 Reflection: Quiz content generated and appended to the lesson file, adhering to style and conciseness guidelines. User confirmation obtained.
