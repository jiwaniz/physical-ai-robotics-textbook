#!/usr/bin/env python3
"""
Seed script for weekly quizzes (12 weeks).
Run: cd backend && python -m scripts.seed_quizzes
"""

import asyncio
import sys
from pathlib import Path

# Add backend directory to path for proper imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy import select

from src.database.connection import AsyncSessionLocal, async_engine
from src.database.models import Base, Question, QuestionCategory, QuestionType, Quiz

SAMPLE_QUIZZES = [
    # ==================== WEEK 1: Introduction to Physical AI ====================
    {
        "week_number": 1,
        "chapter": "Introduction to Physical AI",
        "title": "Week 1: Physical AI Fundamentals",
        "description": "Test your understanding of Physical AI concepts, embodied intelligence, and the robotics landscape.",
        "time_limit_minutes": 15,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What distinguishes Physical AI from traditional AI?",
                    "options": [
                        {"id": "a", "text": "It uses larger neural networks", "is_correct": False},
                        {"id": "b", "text": "It interacts with the physical world through sensors and actuators", "is_correct": True},
                        {"id": "c", "text": "It requires more training data", "is_correct": False},
                        {"id": "d", "text": "It only works on cloud servers", "is_correct": False},
                    ],
                    "explanation": "Physical AI systems are embodied - they perceive and act in the real world, unlike purely digital AI systems.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "Which of the following is NOT a key challenge in humanoid robotics?",
                    "options": [
                        {"id": "a", "text": "Balance and locomotion", "is_correct": False},
                        {"id": "b", "text": "Real-time decision making", "is_correct": False},
                        {"id": "c", "text": "Unlimited computational resources", "is_correct": True},
                        {"id": "d", "text": "Safe human-robot interaction", "is_correct": False},
                    ],
                    "explanation": "Humanoid robots face significant computational constraints. Balance, real-time decisions, and safety are all genuine challenges.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'embodied cognition' in robotics?",
                    "options": [
                        {"id": "a", "text": "Running AI models on embedded hardware", "is_correct": False},
                        {"id": "b", "text": "Intelligence emerging from physical interaction with the environment", "is_correct": True},
                        {"id": "c", "text": "Programming robots using body gestures", "is_correct": False},
                        {"id": "d", "text": "Simulating human organs in robots", "is_correct": False},
                    ],
                    "explanation": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "In the context of robot perception, what does a LiDAR sensor measure?",
                    "options": [
                        {"id": "a", "text": "Temperature of objects", "is_correct": False},
                        {"id": "b", "text": "Distance to objects using laser light", "is_correct": True},
                        {"id": "c", "text": "Color of objects", "is_correct": False},
                        {"id": "d", "text": "Sound waves from the environment", "is_correct": False},
                    ],
                    "explanation": "LiDAR (Light Detection and Ranging) uses laser pulses to measure distances, creating 3D point clouds of the environment.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "A robot's depth camera is returning noisy data near reflective surfaces. What is the most likely cause?",
                    "options": [
                        {"id": "a", "text": "The robot's CPU is overheating", "is_correct": False},
                        {"id": "b", "text": "Infrared light is reflecting unpredictably off shiny surfaces", "is_correct": True},
                        {"id": "c", "text": "The camera lens needs cleaning", "is_correct": False},
                        {"id": "d", "text": "The robot's battery is low", "is_correct": False},
                    ],
                    "explanation": "Depth cameras use infrared patterns. Reflective/shiny surfaces cause IR interference, resulting in noisy depth readings.",
                },
            },
        ],
    },
    # ==================== WEEK 2: Development Environment Setup ====================
    {
        "week_number": 2,
        "chapter": "Development Environment Setup",
        "title": "Week 2: Dev Environment & Tools",
        "description": "Assess your knowledge of setting up robotics development environments, Docker, and essential tools.",
        "time_limit_minutes": 15,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "Why is Docker commonly used in robotics development?",
                    "options": [
                        {"id": "a", "text": "It makes robots run faster", "is_correct": False},
                        {"id": "b", "text": "It provides consistent, reproducible development environments", "is_correct": True},
                        {"id": "c", "text": "It is required by all robotics frameworks", "is_correct": False},
                        {"id": "d", "text": "It replaces the need for simulation", "is_correct": False},
                    ],
                    "explanation": "Docker containers ensure consistent environments across different machines, solving 'it works on my machine' problems.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What is the purpose of 'source /opt/ros/humble/setup.bash' in a terminal?",
                    "options": [
                        {"id": "a", "text": "Installs ROS 2 Humble", "is_correct": False},
                        {"id": "b", "text": "Sets up environment variables for ROS 2 Humble", "is_correct": True},
                        {"id": "c", "text": "Starts the ROS 2 daemon", "is_correct": False},
                        {"id": "d", "text": "Compiles ROS 2 packages", "is_correct": False},
                    ],
                    "explanation": "The setup.bash script sets PATH, PYTHONPATH, and other environment variables needed to use ROS 2 commands and packages.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "You get 'ros2: command not found' after installing ROS 2. What should you check first?",
                    "options": [
                        {"id": "a", "text": "Reinstall ROS 2", "is_correct": False},
                        {"id": "b", "text": "Source the ROS 2 setup script", "is_correct": True},
                        {"id": "c", "text": "Restart your computer", "is_correct": False},
                        {"id": "d", "text": "Update your graphics driver", "is_correct": False},
                    ],
                    "explanation": "The most common cause is not sourcing the setup script. Run 'source /opt/ros/humble/setup.bash' or add it to your .bashrc.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the purpose of a ROS 2 workspace?",
                    "options": [
                        {"id": "a", "text": "To store robot hardware specifications", "is_correct": False},
                        {"id": "b", "text": "To organize and build ROS 2 packages", "is_correct": True},
                        {"id": "c", "text": "To run simulations", "is_correct": False},
                        {"id": "d", "text": "To connect to remote robots", "is_correct": False},
                    ],
                    "explanation": "A workspace is a directory containing ROS 2 packages that can be built together using colcon.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which command is used to build ROS 2 packages in a workspace?",
                    "options": [
                        {"id": "a", "text": "cmake --build .", "is_correct": False},
                        {"id": "b", "text": "colcon build", "is_correct": True},
                        {"id": "c", "text": "ros2 build", "is_correct": False},
                        {"id": "d", "text": "make all", "is_correct": False},
                    ],
                    "explanation": "Colcon is the build tool for ROS 2. 'colcon build' compiles all packages in the workspace.",
                },
            },
        ],
    },
    # ==================== WEEK 3: ROS 2 Architecture ====================
    {
        "week_number": 3,
        "chapter": "ROS 2 Fundamentals",
        "title": "Week 3: ROS 2 Core Concepts",
        "description": "Assess your understanding of ROS 2 architecture, nodes, topics, and basic communication patterns.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the primary communication middleware used by ROS 2?",
                    "options": [
                        {"id": "a", "text": "MQTT", "is_correct": False},
                        {"id": "b", "text": "DDS (Data Distribution Service)", "is_correct": True},
                        {"id": "c", "text": "HTTP/REST", "is_correct": False},
                        {"id": "d", "text": "ZeroMQ", "is_correct": False},
                    ],
                    "explanation": "ROS 2 uses DDS as its middleware, which provides real-time, reliable communication with quality-of-service settings.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "In a ROS 2 Python publisher node, which method is used to send a message?",
                    "options": [
                        {"id": "a", "text": "self.publisher_.send(msg)", "is_correct": False},
                        {"id": "b", "text": "self.publisher_.publish(msg)", "is_correct": True},
                        {"id": "c", "text": "self.publisher_.emit(msg)", "is_correct": False},
                        {"id": "d", "text": "self.publisher_.broadcast(msg)", "is_correct": False},
                    ],
                    "explanation": "In ROS 2, publishers use the publish() method to send messages to a topic.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What does the command 'ros2 topic list' display?",
                    "options": [
                        {"id": "a", "text": "All available ROS 2 packages", "is_correct": False},
                        {"id": "b", "text": "Currently active topics in the ROS 2 system", "is_correct": True},
                        {"id": "c", "text": "All nodes currently running", "is_correct": False},
                        {"id": "d", "text": "The message types defined in the workspace", "is_correct": False},
                    ],
                    "explanation": "ros2 topic list shows all topics currently being published or subscribed to in the ROS 2 graph.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the main difference between ROS 2 Topics and Services?",
                    "options": [
                        {"id": "a", "text": "Topics are faster than Services", "is_correct": False},
                        {"id": "b", "text": "Topics use publish-subscribe for streaming; Services use request-response", "is_correct": True},
                        {"id": "c", "text": "Services can only send strings", "is_correct": False},
                        {"id": "d", "text": "Topics require more memory", "is_correct": False},
                    ],
                    "explanation": "Topics use publish-subscribe for continuous data streaming. Services use request-response for synchronous, one-time operations.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your ROS 2 subscriber is not receiving messages from a publisher. What should you check first?",
                    "options": [
                        {"id": "a", "text": "Verify both nodes use the same topic name and message type", "is_correct": True},
                        {"id": "b", "text": "Restart your computer", "is_correct": False},
                        {"id": "c", "text": "Reinstall ROS 2", "is_correct": False},
                        {"id": "d", "text": "Check if the CPU is overloaded", "is_correct": False},
                    ],
                    "explanation": "Topic name and message type mismatches are the most common cause. Use 'ros2 topic info' to verify.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "In a ROS 2 launch file, how do you remap a topic named 'input' to 'sensor_data'?",
                    "options": [
                        {"id": "a", "text": "remappings=['input:sensor_data']", "is_correct": False},
                        {"id": "b", "text": "remappings=[('input', 'sensor_data')]", "is_correct": True},
                        {"id": "c", "text": "remap={'input': 'sensor_data'}", "is_correct": False},
                        {"id": "d", "text": "topics={'input': 'sensor_data'}", "is_correct": False},
                    ],
                    "explanation": "ROS 2 launch file remappings use a list of tuples: [('original_name', 'new_name')].",
                },
            },
        ],
    },
    # ==================== WEEK 4: Nodes & Topics ====================
    {
        "week_number": 4,
        "chapter": "Nodes and Topics",
        "title": "Week 4: ROS 2 Communication Patterns",
        "description": "Test your knowledge of ROS 2 nodes, topics, publishers, subscribers, and QoS settings.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is a ROS 2 node?",
                    "options": [
                        {"id": "a", "text": "A physical connection point on a robot", "is_correct": False},
                        {"id": "b", "text": "An independent executable that performs computation in ROS 2", "is_correct": True},
                        {"id": "c", "text": "A type of sensor", "is_correct": False},
                        {"id": "d", "text": "A configuration file", "is_correct": False},
                    ],
                    "explanation": "A node is a process that performs computation. Robots typically have many nodes for different functions.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is QoS (Quality of Service) in ROS 2?",
                    "options": [
                        {"id": "a", "text": "A metric for measuring robot performance", "is_correct": False},
                        {"id": "b", "text": "Policies that control reliability, durability, and history of communication", "is_correct": True},
                        {"id": "c", "text": "A debugging tool", "is_correct": False},
                        {"id": "d", "text": "A security protocol", "is_correct": False},
                    ],
                    "explanation": "QoS policies configure how messages are delivered, including reliability (best effort vs reliable), durability, and queue sizes.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which QoS profile is typically used for sensor data that tolerates occasional message loss?",
                    "options": [
                        {"id": "a", "text": "RELIABLE", "is_correct": False},
                        {"id": "b", "text": "BEST_EFFORT", "is_correct": True},
                        {"id": "c", "text": "TRANSIENT_LOCAL", "is_correct": False},
                        {"id": "d", "text": "KEEP_ALL", "is_correct": False},
                    ],
                    "explanation": "BEST_EFFORT is used for high-frequency sensor data where occasional loss is acceptable and low latency is important.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "A late-joining subscriber misses the first message from a publisher. Which QoS setting could fix this?",
                    "options": [
                        {"id": "a", "text": "Set reliability to BEST_EFFORT", "is_correct": False},
                        {"id": "b", "text": "Set durability to TRANSIENT_LOCAL", "is_correct": True},
                        {"id": "c", "text": "Increase the history depth", "is_correct": False},
                        {"id": "d", "text": "Use a different topic name", "is_correct": False},
                    ],
                    "explanation": "TRANSIENT_LOCAL durability keeps messages for late-joining subscribers. The publisher stores recent messages.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What command shows the data being published on a topic?",
                    "options": [
                        {"id": "a", "text": "ros2 topic info /topic_name", "is_correct": False},
                        {"id": "b", "text": "ros2 topic echo /topic_name", "is_correct": True},
                        {"id": "c", "text": "ros2 topic list", "is_correct": False},
                        {"id": "d", "text": "ros2 topic type /topic_name", "is_correct": False},
                    ],
                    "explanation": "'ros2 topic echo' subscribes to a topic and prints the messages to the terminal.",
                },
            },
        ],
    },
    # ==================== WEEK 5: Services & Actions ====================
    {
        "week_number": 5,
        "chapter": "Services and Actions",
        "title": "Week 5: Services & Actions",
        "description": "Test your understanding of ROS 2 services for synchronous calls and actions for long-running tasks.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "When should you use a ROS 2 Service instead of a Topic?",
                    "options": [
                        {"id": "a", "text": "For continuous sensor data streams", "is_correct": False},
                        {"id": "b", "text": "For request-response operations that need a reply", "is_correct": True},
                        {"id": "c", "text": "For broadcasting to multiple nodes", "is_correct": False},
                        {"id": "d", "text": "For high-frequency data", "is_correct": False},
                    ],
                    "explanation": "Services are ideal for request-response patterns where you need a confirmed result, like getting a parameter or triggering an action.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What makes ROS 2 Actions different from Services?",
                    "options": [
                        {"id": "a", "text": "Actions are faster", "is_correct": False},
                        {"id": "b", "text": "Actions provide feedback during execution and can be canceled", "is_correct": True},
                        {"id": "c", "text": "Services can handle multiple requests simultaneously", "is_correct": False},
                        {"id": "d", "text": "Actions don't require a response", "is_correct": False},
                    ],
                    "explanation": "Actions support long-running tasks with progress feedback, the ability to cancel, and a final result.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which components make up a ROS 2 Action?",
                    "options": [
                        {"id": "a", "text": "Request and Response", "is_correct": False},
                        {"id": "b", "text": "Goal, Result, and Feedback", "is_correct": True},
                        {"id": "c", "text": "Publisher and Subscriber", "is_correct": False},
                        {"id": "d", "text": "Input and Output", "is_correct": False},
                    ],
                    "explanation": "Actions have three parts: Goal (what to do), Feedback (progress updates), and Result (final outcome).",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your service call hangs indefinitely. What is the most likely cause?",
                    "options": [
                        {"id": "a", "text": "The service server is not running", "is_correct": True},
                        {"id": "b", "text": "The topic is full", "is_correct": False},
                        {"id": "c", "text": "The message type is too large", "is_correct": False},
                        {"id": "d", "text": "DDS is not installed", "is_correct": False},
                    ],
                    "explanation": "Service calls block until a response. If the server isn't running, the call waits forever. Use 'ros2 service list' to check.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which command lists all available services in ROS 2?",
                    "options": [
                        {"id": "a", "text": "ros2 service info", "is_correct": False},
                        {"id": "b", "text": "ros2 service list", "is_correct": True},
                        {"id": "c", "text": "ros2 srv list", "is_correct": False},
                        {"id": "d", "text": "ros2 node services", "is_correct": False},
                    ],
                    "explanation": "'ros2 service list' shows all services currently available in the ROS 2 system.",
                },
            },
        ],
    },
    # ==================== WEEK 6: URDF & Gazebo ====================
    {
        "week_number": 6,
        "chapter": "Gazebo Simulation",
        "title": "Week 6: Gazebo & URDF Basics",
        "description": "Test your knowledge of Gazebo simulation, URDF robot descriptions, and simulation setup.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What does URDF stand for?",
                    "options": [
                        {"id": "a", "text": "Universal Robot Definition Format", "is_correct": False},
                        {"id": "b", "text": "Unified Robot Description Format", "is_correct": True},
                        {"id": "c", "text": "Universal Robotics Data File", "is_correct": False},
                        {"id": "d", "text": "Unified Robotic Design Framework", "is_correct": False},
                    ],
                    "explanation": "URDF (Unified Robot Description Format) is an XML format for describing robot models in ROS.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which attributes are required to define a cylinder geometry in URDF?",
                    "options": [
                        {"id": "a", "text": "width and height", "is_correct": False},
                        {"id": "b", "text": "radius and length", "is_correct": True},
                        {"id": "c", "text": "diameter and depth", "is_correct": False},
                        {"id": "d", "text": "size and scale", "is_correct": False},
                    ],
                    "explanation": "URDF cylinders require 'radius' (cross-section) and 'length' (height) attributes.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your robot model in Gazebo falls through the ground. What is the most likely cause?",
                    "options": [
                        {"id": "a", "text": "The visual geometry is incorrect", "is_correct": False},
                        {"id": "b", "text": "The collision geometry is missing or incorrect", "is_correct": True},
                        {"id": "c", "text": "The robot name is too long", "is_correct": False},
                        {"id": "d", "text": "Gazebo needs more RAM", "is_correct": False},
                    ],
                    "explanation": "Physics simulation uses collision geometry. Missing collision elements cause objects to fall through surfaces.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the difference between <visual> and <collision> elements in URDF?",
                    "options": [
                        {"id": "a", "text": "Visual is for 2D, collision is for 3D", "is_correct": False},
                        {"id": "b", "text": "Visual defines appearance; collision defines physics geometry", "is_correct": True},
                        {"id": "c", "text": "Visual is required, collision is optional", "is_correct": False},
                        {"id": "d", "text": "They are identical and interchangeable", "is_correct": False},
                    ],
                    "explanation": "Visual elements define rendering. Collision elements define physics geometry, often simplified for performance.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which URDF element connects two links and defines their relative motion?",
                    "options": [
                        {"id": "a", "text": "<connection>", "is_correct": False},
                        {"id": "b", "text": "<joint>", "is_correct": True},
                        {"id": "c", "text": "<link>", "is_correct": False},
                        {"id": "d", "text": "<transform>", "is_correct": False},
                    ],
                    "explanation": "Joints connect links and define their kinematic relationship (fixed, revolute, prismatic, etc.).",
                },
            },
        ],
    },
    # ==================== WEEK 7: Sensors & Perception ====================
    {
        "week_number": 7,
        "chapter": "Sensors and Perception",
        "title": "Week 7: Sensors & Perception",
        "description": "Test your knowledge of robot sensors, perception algorithms, and sensor data processing.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is sensor fusion in robotics?",
                    "options": [
                        {"id": "a", "text": "Physically combining multiple sensors into one", "is_correct": False},
                        {"id": "b", "text": "Combining data from multiple sensors for better perception", "is_correct": True},
                        {"id": "c", "text": "Using a single sensor for all tasks", "is_correct": False},
                        {"id": "d", "text": "Converting sensor data to video", "is_correct": False},
                    ],
                    "explanation": "Sensor fusion combines data from multiple sensors to get more accurate and robust perception than any single sensor provides.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the main advantage of RGB-D cameras over standard RGB cameras?",
                    "options": [
                        {"id": "a", "text": "Higher color resolution", "is_correct": False},
                        {"id": "b", "text": "They provide depth information along with color", "is_correct": True},
                        {"id": "c", "text": "They work better in low light", "is_correct": False},
                        {"id": "d", "text": "They are cheaper", "is_correct": False},
                    ],
                    "explanation": "RGB-D cameras capture both color (RGB) and depth (D) data, enabling 3D perception without additional sensors.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What ROS 2 message type is commonly used for 3D point cloud data?",
                    "options": [
                        {"id": "a", "text": "sensor_msgs/msg/Image", "is_correct": False},
                        {"id": "b", "text": "sensor_msgs/msg/PointCloud2", "is_correct": True},
                        {"id": "c", "text": "geometry_msgs/msg/Point", "is_correct": False},
                        {"id": "d", "text": "std_msgs/msg/Float32MultiArray", "is_correct": False},
                    ],
                    "explanation": "PointCloud2 is the standard message type for 3D point cloud data from LiDAR and depth cameras.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your robot's IMU readings are drifting over time. What is the most likely cause?",
                    "options": [
                        {"id": "a", "text": "The IMU needs calibration", "is_correct": True},
                        {"id": "b", "text": "The CPU is overheating", "is_correct": False},
                        {"id": "c", "text": "The battery is low", "is_correct": False},
                        {"id": "d", "text": "The topic is congested", "is_correct": False},
                    ],
                    "explanation": "IMU sensors accumulate errors over time (drift). Proper calibration and sensor fusion with other sensors can mitigate this.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "Which sensor is best suited for measuring precise distances in outdoor environments?",
                    "options": [
                        {"id": "a", "text": "Infrared depth camera", "is_correct": False},
                        {"id": "b", "text": "LiDAR", "is_correct": True},
                        {"id": "c", "text": "Ultrasonic sensor", "is_correct": False},
                        {"id": "d", "text": "Webcam", "is_correct": False},
                    ],
                    "explanation": "LiDAR works well outdoors regardless of lighting conditions and provides accurate distance measurements over long ranges.",
                },
            },
        ],
    },
    # ==================== WEEK 8: Isaac Sim Introduction ====================
    {
        "week_number": 8,
        "chapter": "Isaac Sim Introduction",
        "title": "Week 8: NVIDIA Isaac Sim",
        "description": "Test your understanding of NVIDIA Isaac Sim for robotics simulation and synthetic data generation.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is NVIDIA Isaac Sim built on top of?",
                    "options": [
                        {"id": "a", "text": "Unity Engine", "is_correct": False},
                        {"id": "b", "text": "NVIDIA Omniverse platform", "is_correct": True},
                        {"id": "c", "text": "Unreal Engine", "is_correct": False},
                        {"id": "d", "text": "Gazebo", "is_correct": False},
                    ],
                    "explanation": "Isaac Sim is built on NVIDIA Omniverse, which provides photorealistic rendering and physics simulation capabilities.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is a key advantage of Isaac Sim over traditional simulators like Gazebo?",
                    "options": [
                        {"id": "a", "text": "It is open source", "is_correct": False},
                        {"id": "b", "text": "Photorealistic rendering and GPU-accelerated physics", "is_correct": True},
                        {"id": "c", "text": "It requires less computational resources", "is_correct": False},
                        {"id": "d", "text": "It only supports ROS 1", "is_correct": False},
                    ],
                    "explanation": "Isaac Sim provides photorealistic visuals for training vision models and GPU-accelerated physics for faster simulation.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What file format does Isaac Sim use for scene descriptions?",
                    "options": [
                        {"id": "a", "text": "URDF only", "is_correct": False},
                        {"id": "b", "text": "USD (Universal Scene Description)", "is_correct": True},
                        {"id": "c", "text": "SDF only", "is_correct": False},
                        {"id": "d", "text": "FBX", "is_correct": False},
                    ],
                    "explanation": "Isaac Sim uses USD (Universal Scene Description) format, which supports complex scenes and can import URDF models.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Isaac Sim is running very slowly. What should you check first?",
                    "options": [
                        {"id": "a", "text": "Your internet connection", "is_correct": False},
                        {"id": "b", "text": "GPU drivers and NVIDIA GPU availability", "is_correct": True},
                        {"id": "c", "text": "The ROS 2 version", "is_correct": False},
                        {"id": "d", "text": "The Python version", "is_correct": False},
                    ],
                    "explanation": "Isaac Sim requires a capable NVIDIA GPU. Performance issues often relate to GPU drivers or insufficient GPU memory.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What does 'domain randomization' refer to in simulation?",
                    "options": [
                        {"id": "a", "text": "Randomly selecting which simulator to use", "is_correct": False},
                        {"id": "b", "text": "Varying visual and physical parameters to improve real-world transfer", "is_correct": True},
                        {"id": "c", "text": "Randomly starting the robot in different positions", "is_correct": False},
                        {"id": "d", "text": "Using random network architectures", "is_correct": False},
                    ],
                    "explanation": "Domain randomization varies textures, lighting, physics, and other parameters during training to help models generalize to the real world.",
                },
            },
        ],
    },
    # ==================== WEEK 9: Synthetic Data Generation ====================
    {
        "week_number": 9,
        "chapter": "Synthetic Data Generation",
        "title": "Week 9: Synthetic Data for AI",
        "description": "Test your knowledge of generating synthetic data for training AI models in robotics.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "Why is synthetic data valuable for training robotics AI models?",
                    "options": [
                        {"id": "a", "text": "It is always more accurate than real data", "is_correct": False},
                        {"id": "b", "text": "It can be generated at scale with perfect labels at low cost", "is_correct": True},
                        {"id": "c", "text": "It requires no simulation setup", "is_correct": False},
                        {"id": "d", "text": "It eliminates the need for testing on real robots", "is_correct": False},
                    ],
                    "explanation": "Synthetic data provides unlimited, perfectly labeled training data without manual annotation costs or safety concerns.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'the sim-to-real gap'?",
                    "options": [
                        {"id": "a", "text": "The price difference between simulators", "is_correct": False},
                        {"id": "b", "text": "The performance difference when transferring models from simulation to real world", "is_correct": True},
                        {"id": "c", "text": "The time it takes to set up a simulator", "is_correct": False},
                        {"id": "d", "text": "The distance between simulated objects", "is_correct": False},
                    ],
                    "explanation": "The sim-to-real gap refers to the performance degradation when models trained in simulation are deployed on real hardware.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "Which type of annotation is automatically available in synthetic data that requires manual effort for real data?",
                    "options": [
                        {"id": "a", "text": "File names", "is_correct": False},
                        {"id": "b", "text": "Pixel-perfect segmentation masks and depth maps", "is_correct": True},
                        {"id": "c", "text": "Image resolution", "is_correct": False},
                        {"id": "d", "text": "Timestamp information", "is_correct": False},
                    ],
                    "explanation": "Simulators can automatically generate perfect segmentation masks, depth, normals, and object poses - expensive to annotate manually.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your model trained on synthetic data performs poorly on real images. What should you try first?",
                    "options": [
                        {"id": "a", "text": "Use a larger neural network", "is_correct": False},
                        {"id": "b", "text": "Apply domain randomization to increase visual diversity", "is_correct": True},
                        {"id": "c", "text": "Train for more epochs", "is_correct": False},
                        {"id": "d", "text": "Use a different programming language", "is_correct": False},
                    ],
                    "explanation": "Domain randomization helps bridge the sim-to-real gap by exposing the model to more visual variations during training.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is NVIDIA Replicator used for?",
                    "options": [
                        {"id": "a", "text": "Copying files between computers", "is_correct": False},
                        {"id": "b", "text": "Generating synthetic data with domain randomization", "is_correct": True},
                        {"id": "c", "text": "Training neural networks", "is_correct": False},
                        {"id": "d", "text": "Running ROS 2 nodes", "is_correct": False},
                    ],
                    "explanation": "Replicator is a synthetic data generation toolkit in Isaac Sim that automates domain randomization and data capture.",
                },
            },
        ],
    },
    # ==================== WEEK 10: Sim-to-Real Transfer ====================
    {
        "week_number": 10,
        "chapter": "Sim-to-Real Transfer",
        "title": "Week 10: Sim-to-Real Transfer",
        "description": "Test your understanding of techniques for transferring learned behaviors from simulation to real robots.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the primary goal of sim-to-real transfer?",
                    "options": [
                        {"id": "a", "text": "To make simulations run faster", "is_correct": False},
                        {"id": "b", "text": "To deploy models trained in simulation on real robots", "is_correct": True},
                        {"id": "c", "text": "To visualize robot movements", "is_correct": False},
                        {"id": "d", "text": "To reduce simulation costs", "is_correct": False},
                    ],
                    "explanation": "Sim-to-real transfer aims to successfully deploy AI models trained in simulation to work reliably on physical robots.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "Which technique involves training with varied simulation parameters to improve real-world robustness?",
                    "options": [
                        {"id": "a", "text": "Transfer learning", "is_correct": False},
                        {"id": "b", "text": "Domain randomization", "is_correct": True},
                        {"id": "c", "text": "Fine-tuning", "is_correct": False},
                        {"id": "d", "text": "Data augmentation", "is_correct": False},
                    ],
                    "explanation": "Domain randomization varies simulation parameters (lighting, textures, physics) so models learn to be robust to variations.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'system identification' in the context of sim-to-real?",
                    "options": [
                        {"id": "a", "text": "Identifying which computer runs the simulation", "is_correct": False},
                        {"id": "b", "text": "Measuring real robot parameters to match the simulation", "is_correct": True},
                        {"id": "c", "text": "Naming robots in the fleet", "is_correct": False},
                        {"id": "d", "text": "Identifying bugs in the code", "is_correct": False},
                    ],
                    "explanation": "System identification measures physical parameters (mass, friction, delays) of the real robot to tune the simulation.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your robot control policy works in simulation but oscillates on the real robot. What is likely the cause?",
                    "options": [
                        {"id": "a", "text": "The neural network is too large", "is_correct": False},
                        {"id": "b", "text": "Latency or dynamics differences between sim and real", "is_correct": True},
                        {"id": "c", "text": "The robot battery is low", "is_correct": False},
                        {"id": "d", "text": "The simulation framerate is too high", "is_correct": False},
                    ],
                    "explanation": "Control policies are sensitive to timing and dynamics. Real-world latency and different physical properties can cause instability.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'domain adaptation' in sim-to-real transfer?",
                    "options": [
                        {"id": "a", "text": "Changing the simulation software", "is_correct": False},
                        {"id": "b", "text": "Learning to map between simulation and real-world distributions", "is_correct": True},
                        {"id": "c", "text": "Adapting to different robot types", "is_correct": False},
                        {"id": "d", "text": "Changing the programming domain", "is_correct": False},
                    ],
                    "explanation": "Domain adaptation learns transformations to align simulation data distribution with real-world data distribution.",
                },
            },
        ],
    },
    # ==================== WEEK 11: VLA Foundations ====================
    {
        "week_number": 11,
        "chapter": "VLA Foundations",
        "title": "Week 11: Vision-Language-Action Models",
        "description": "Test your understanding of Vision-Language-Action (VLA) models for robot learning.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What does VLA stand for in robotics AI?",
                    "options": [
                        {"id": "a", "text": "Virtual Learning Agent", "is_correct": False},
                        {"id": "b", "text": "Vision-Language-Action", "is_correct": True},
                        {"id": "c", "text": "Visual Localization Algorithm", "is_correct": False},
                        {"id": "d", "text": "Variable Length Array", "is_correct": False},
                    ],
                    "explanation": "VLA models combine visual perception, language understanding, and action generation for robot control.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is the key advantage of VLA models over traditional robot programming?",
                    "options": [
                        {"id": "a", "text": "They run faster on CPUs", "is_correct": False},
                        {"id": "b", "text": "They can understand natural language instructions", "is_correct": True},
                        {"id": "c", "text": "They require less memory", "is_correct": False},
                        {"id": "d", "text": "They don't need training data", "is_correct": False},
                    ],
                    "explanation": "VLAs can follow natural language commands, making robots more accessible and flexible without explicit programming.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What type of input does a VLA model typically receive?",
                    "options": [
                        {"id": "a", "text": "Only text commands", "is_correct": False},
                        {"id": "b", "text": "Camera images and natural language instructions", "is_correct": True},
                        {"id": "c", "text": "Only joint positions", "is_correct": False},
                        {"id": "d", "text": "Only depth sensor data", "is_correct": False},
                    ],
                    "explanation": "VLAs combine visual input (images/video) with language instructions to generate robot actions.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'imitation learning' in the context of VLA models?",
                    "options": [
                        {"id": "a", "text": "Copying another AI model's architecture", "is_correct": False},
                        {"id": "b", "text": "Learning from demonstrations of desired behavior", "is_correct": True},
                        {"id": "c", "text": "Imitating human speech patterns", "is_correct": False},
                        {"id": "d", "text": "Copying robot firmware", "is_correct": False},
                    ],
                    "explanation": "Imitation learning trains models by showing examples of correct behavior, often through teleoperation demonstrations.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your VLA model works well on trained tasks but fails on slightly different objects. What should you try?",
                    "options": [
                        {"id": "a", "text": "Use a smaller model", "is_correct": False},
                        {"id": "b", "text": "Collect more diverse training demonstrations", "is_correct": True},
                        {"id": "c", "text": "Remove the language component", "is_correct": False},
                        {"id": "d", "text": "Reduce the camera resolution", "is_correct": False},
                    ],
                    "explanation": "VLA generalization improves with more diverse training data covering variations in objects, scenes, and tasks.",
                },
            },
        ],
    },
    # ==================== WEEK 12: Advanced VLA & Deployment ====================
    {
        "week_number": 12,
        "chapter": "Advanced VLA and Deployment",
        "title": "Week 12: Advanced VLA & Real-World Deployment",
        "description": "Test your knowledge of advanced VLA techniques and deploying AI models on real robots.",
        "time_limit_minutes": 20,
        "max_attempts": 2,
        "passing_score": 60.0,
        "is_published": True,
        "questions": [
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'action chunking' in VLA models?",
                    "options": [
                        {"id": "a", "text": "Dividing actions into smaller pieces", "is_correct": False},
                        {"id": "b", "text": "Predicting multiple future actions at once", "is_correct": True},
                        {"id": "c", "text": "Compressing action data for storage", "is_correct": False},
                        {"id": "d", "text": "Grouping similar actions together", "is_correct": False},
                    ],
                    "explanation": "Action chunking predicts sequences of actions, improving temporal coherence and reducing inference frequency requirements.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "Why is inference speed important for VLA models on real robots?",
                    "options": [
                        {"id": "a", "text": "It affects the robot's battery life", "is_correct": False},
                        {"id": "b", "text": "Real-time control requires fast action predictions", "is_correct": True},
                        {"id": "c", "text": "It determines the robot's maximum speed", "is_correct": False},
                        {"id": "d", "text": "It affects WiFi connectivity", "is_correct": False},
                    ],
                    "explanation": "Robots need to respond quickly to changing situations. Slow inference creates dangerous delays in reactive control.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What is model quantization used for in robot deployment?",
                    "options": [
                        {"id": "a", "text": "Measuring model accuracy", "is_correct": False},
                        {"id": "b", "text": "Reducing model size and inference time by using lower precision", "is_correct": True},
                        {"id": "c", "text": "Counting the number of parameters", "is_correct": False},
                        {"id": "d", "text": "Converting models between frameworks", "is_correct": False},
                    ],
                    "explanation": "Quantization converts weights to lower precision (e.g., FP32 to INT8), reducing memory and speeding up inference on edge devices.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your deployed VLA model's actions are delayed and jerky. What should you investigate?",
                    "options": [
                        {"id": "a", "text": "The robot's paint color", "is_correct": False},
                        {"id": "b", "text": "Model inference time and hardware acceleration", "is_correct": True},
                        {"id": "c", "text": "The training dataset size", "is_correct": False},
                        {"id": "d", "text": "The robot's WiFi password", "is_correct": False},
                    ],
                    "explanation": "Jerky motion often indicates inference bottlenecks. Check if GPU acceleration is enabled and consider model optimization.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What safety consideration is most important when deploying VLA models?",
                    "options": [
                        {"id": "a", "text": "Model file size", "is_correct": False},
                        {"id": "b", "text": "Fail-safe mechanisms for unexpected model behavior", "is_correct": True},
                        {"id": "c", "text": "The color of the robot", "is_correct": False},
                        {"id": "d", "text": "Internet connectivity", "is_correct": False},
                    ],
                    "explanation": "VLA models can produce unexpected outputs. Emergency stops, force limits, and monitoring systems are essential for safety.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 2.0,
                "content": {
                    "question_text": "What is 'continual learning' in deployed robot systems?",
                    "options": [
                        {"id": "a", "text": "Running training constantly at high GPU usage", "is_correct": False},
                        {"id": "b", "text": "Updating the model with new data from real-world deployment", "is_correct": True},
                        {"id": "c", "text": "Learning multiple languages simultaneously", "is_correct": False},
                        {"id": "d", "text": "Copying weights from other robots", "is_correct": False},
                    ],
                    "explanation": "Continual learning allows robots to improve from real-world experience while maintaining previous capabilities.",
                },
            },
        ],
    },
]


async def seed_quizzes():
    """Seed the database with weekly quizzes for all 12 weeks."""
    # Create tables using the Base from models.py
    async with async_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    print("Database tables created/verified.")

    async with AsyncSessionLocal() as session:
        for quiz_data in SAMPLE_QUIZZES:
            # Check if quiz already exists
            result = await session.execute(
                select(Quiz).where(Quiz.week_number == quiz_data["week_number"])
            )
            existing = result.scalar_one_or_none()

            if existing:
                print(f"Quiz for week {quiz_data['week_number']} already exists, skipping...")
                continue

            # Create quiz
            questions_data = quiz_data.pop("questions")
            quiz = Quiz(**quiz_data)
            session.add(quiz)
            await session.flush()

            # Add questions
            for idx, q_data in enumerate(questions_data):
                question = Question(
                    quiz_id=quiz.id,
                    order_index=idx + 1,
                    **q_data,
                )
                session.add(question)

            await session.commit()
            print(f"Created quiz: {quiz.title} with {len(questions_data)} questions")

    print("\nSeeding complete! Created quizzes for all 12 weeks.")


if __name__ == "__main__":
    asyncio.run(seed_quizzes())
