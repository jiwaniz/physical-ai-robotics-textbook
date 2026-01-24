#!/usr/bin/env python3
"""
Seed script for sample quizzes.
Run: python -m scripts.seed_quizzes
"""

import asyncio
import sys
from pathlib import Path

# Add backend src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from sqlalchemy import select

from database.connection import async_session_maker, init_db
from database.models import Question, QuestionCategory, QuestionType, Quiz

SAMPLE_QUIZZES = [
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
                        {
                            "id": "b",
                            "text": "It interacts with the physical world through sensors and actuators",
                            "is_correct": True,
                        },
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
                        {
                            "id": "c",
                            "text": "Unlimited computational resources",
                            "is_correct": True,
                        },
                        {"id": "d", "text": "Safe human-robot interaction", "is_correct": False},
                    ],
                    "explanation": "Humanoid robots face significant computational constraints. Balance, real-time decisions, and safety are all genuine challenges.",
                },
            },
            {
                "question_type": QuestionType.SHORT_ANSWER,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 3.0,
                "content": {
                    "question_text": "Explain the concept of 'embodied cognition' and why it's important for robotics.",
                    "expected_keywords": [
                        "body",
                        "environment",
                        "interact",
                        "physical",
                        "cognition",
                        "grounded",
                    ],
                    "max_length": 300,
                    "sample_answer": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world. For robotics, this means intelligence emerges from physical interaction with the environment, not just abstract computation.",
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
                        {
                            "id": "b",
                            "text": "Distance to objects using laser light",
                            "is_correct": True,
                        },
                        {"id": "c", "text": "Color of objects", "is_correct": False},
                        {
                            "id": "d",
                            "text": "Sound waves from the environment",
                            "is_correct": False,
                        },
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
                        {
                            "id": "b",
                            "text": "Infrared light is reflecting unpredictably off shiny surfaces",
                            "is_correct": True,
                        },
                        {"id": "c", "text": "The camera lens needs cleaning", "is_correct": False},
                        {"id": "d", "text": "The robot's battery is low", "is_correct": False},
                    ],
                    "explanation": "Depth cameras (like Intel RealSense) use infrared patterns. Reflective/shiny surfaces cause IR interference, resulting in noisy depth readings.",
                },
            },
        ],
    },
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
                "question_type": QuestionType.CODE_COMPLETION,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 3.0,
                "content": {
                    "question_text": "Complete the ROS 2 Python publisher node to publish a String message.",
                    "code_template": """import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        # Fill in the blank to publish the message
        _______________

def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()""",
                    "language": "python",
                    "expected_solution": "self.publisher_.publish(msg)",
                    "hints": [
                        "Use the publisher object created in __init__",
                        "The method name matches what publishers do",
                    ],
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 2.0,
                "content": {
                    "question_text": "What does the command `ros2 topic list` display?",
                    "options": [
                        {"id": "a", "text": "All available ROS 2 packages", "is_correct": False},
                        {
                            "id": "b",
                            "text": "Currently active topics in the ROS 2 system",
                            "is_correct": True,
                        },
                        {"id": "c", "text": "All nodes currently running", "is_correct": False},
                        {
                            "id": "d",
                            "text": "The message types defined in the workspace",
                            "is_correct": False,
                        },
                    ],
                    "explanation": "ros2 topic list shows all topics currently being published or subscribed to in the ROS 2 graph.",
                },
            },
            {
                "question_type": QuestionType.SHORT_ANSWER,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 3.0,
                "content": {
                    "question_text": "Explain the difference between ROS 2 Topics and Services. When would you use each?",
                    "expected_keywords": [
                        "publish",
                        "subscribe",
                        "request",
                        "response",
                        "async",
                        "sync",
                        "streaming",
                        "one-time",
                    ],
                    "max_length": 400,
                    "sample_answer": "Topics use publish-subscribe for continuous, asynchronous data streaming (e.g., sensor data). Services use request-response for synchronous, one-time operations (e.g., triggering an action). Use topics for ongoing data flow, services for discrete commands.",
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your ROS 2 subscriber node is not receiving messages. The publisher shows messages being sent. What should you check first?",
                    "options": [
                        {
                            "id": "a",
                            "text": "Verify both nodes are using the same topic name and message type",
                            "is_correct": True,
                        },
                        {"id": "b", "text": "Restart your computer", "is_correct": False},
                        {"id": "c", "text": "Reinstall ROS 2", "is_correct": False},
                        {"id": "d", "text": "Check if the CPU is overloaded", "is_correct": False},
                    ],
                    "explanation": "Topic name and message type mismatches are the most common cause. Use 'ros2 topic info' to verify both ends match.",
                },
            },
            {
                "question_type": QuestionType.CODE_COMPLETION,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 3.0,
                "content": {
                    "question_text": "Complete the launch file to include a node from the 'my_package' package.",
                    "code_template": """from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_instance',
            # Fill in to remap the topic 'input' to 'sensor_data'
            remappings=[_______________]
        ),
    ])""",
                    "language": "python",
                    "expected_solution": "('input', 'sensor_data')",
                    "hints": [
                        "Remappings are tuples of (original, new)",
                        "The format is ('from_topic', 'to_topic')",
                    ],
                },
            },
        ],
    },
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
                        {
                            "id": "a",
                            "text": "Universal Robot Definition Format",
                            "is_correct": False,
                        },
                        {"id": "b", "text": "Unified Robot Description Format", "is_correct": True},
                        {"id": "c", "text": "Universal Robotics Data File", "is_correct": False},
                        {
                            "id": "d",
                            "text": "Unified Robotic Design Framework",
                            "is_correct": False,
                        },
                    ],
                    "explanation": "URDF (Unified Robot Description Format) is an XML format for describing robot models in ROS.",
                },
            },
            {
                "question_type": QuestionType.CODE_COMPLETION,
                "category": QuestionCategory.CODE_COMPREHENSION,
                "points": 3.0,
                "content": {
                    "question_text": "Complete the URDF to define a cylindrical link for a robot arm segment.",
                    "code_template": """<link name="arm_link">
  <visual>
    <geometry>
      <cylinder _______ _______/>
    </geometry>
  </visual>
</link>""",
                    "language": "xml",
                    "expected_solution": 'radius="0.05" length="0.3"',
                    "hints": [
                        "Cylinders need two dimensions",
                        "Think about the cross-section and height",
                    ],
                },
            },
            {
                "question_type": QuestionType.MULTIPLE_CHOICE,
                "category": QuestionCategory.TROUBLESHOOTING,
                "points": 2.0,
                "content": {
                    "question_text": "Your robot model in Gazebo falls through the ground plane. What is the most likely cause?",
                    "options": [
                        {
                            "id": "a",
                            "text": "The visual geometry is incorrect",
                            "is_correct": False,
                        },
                        {
                            "id": "b",
                            "text": "The collision geometry is missing or incorrect",
                            "is_correct": True,
                        },
                        {"id": "c", "text": "The robot name is too long", "is_correct": False},
                        {"id": "d", "text": "Gazebo needs more RAM", "is_correct": False},
                    ],
                    "explanation": "Physics simulation uses collision geometry, not visual geometry. Missing or zero-sized collision elements cause objects to fall through surfaces.",
                },
            },
            {
                "question_type": QuestionType.SHORT_ANSWER,
                "category": QuestionCategory.CONCEPTUAL,
                "points": 3.0,
                "content": {
                    "question_text": "What is the difference between the <visual> and <collision> elements in URDF?",
                    "expected_keywords": [
                        "visual",
                        "rendering",
                        "display",
                        "collision",
                        "physics",
                        "simplified",
                    ],
                    "max_length": 300,
                    "sample_answer": "Visual elements define how the robot appears (rendering/display). Collision elements define the geometry used for physics calculations. Collision meshes are often simplified for performance while visuals can be detailed.",
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
]


async def seed_quizzes():
    """Seed the database with sample quizzes."""
    await init_db()

    async with async_session_maker() as session:
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

    print("\nSeeding complete!")


if __name__ == "__main__":
    asyncio.run(seed_quizzes())
