# Week 11: Vision-Language-Action Foundations

## Overview

Welcome to the final module! This week introduces Vision-Language-Action (VLA) models, the cutting-edge of Physical AI. You'll learn how multimodal transformers enable robots to understand visual scenes, interpret natural language commands, and generate appropriate actions.

## Learning Objectives

By the end of this week, you will be able to:

- Understand the VLA model architecture and its components
- Explain how vision, language, and action modalities are integrated
- Implement a simple VLA pipeline with pre-trained models
- Fine-tune VLA models for specific robotic tasks
- Integrate VLA models with ROS 2 and simulators
- Evaluate VLA model performance quantitatively

## What are VLA Models?

**Vision-Language-Action (VLA)** models are multimodal transformers that map:
- **Vision** (camera images, depth, point clouds)
- **Language** (natural language instructions)
→ **Actions** (robot motor commands, trajectories)

### Why VLA Models?

**Traditional robotics pipeline:**
```
Perception → State Estimation → Planning → Control
(separate modules, hand-crafted interfaces)
```

**VLA approach:**
```
Vision + Language → Transformer → Actions
(end-to-end learning, unified representation)
```

**Advantages:**
- **Generalization**: Single model handles diverse tasks
- **Natural interaction**: Accept language instructions
- **Transfer learning**: Leverage internet-scale vision-language data
- **Simplicity**: No hand-crafted perception/planning modules

### Real-World VLA Systems

| Model | Organization | Key Achievement |
|-------|-------------|-----------------|
| **RT-1** | Google DeepMind | 700 tasks on real robots |
| **RT-2** | Google DeepMind | Vision-language model → actions |
| **PaLM-E** | Google | 540B parameter embodied AI |
| **RoboFlamingo** | Open source | Open-weights VLA |
| **OpenVLA** | Open source | 7B parameter open model |

## VLA Architecture

### High-Level Structure

```
┌─────────────┐     ┌─────────────┐
│   Camera    │────▶│   Vision    │
│   Image     │     │   Encoder   │
└─────────────┘     └──────┬──────┘
                           │
                           ▼
                    ┌─────────────┐
┌─────────────┐     │             │     ┌─────────────┐
│  Language   │────▶│ Transformer │────▶│   Action    │
│ Instruction │     │    Core     │     │   Decoder   │
└─────────────┘     │             │     └──────┬──────┘
                    └─────────────┘            │
                                               ▼
                                        ┌─────────────┐
                                        │   Robot     │
                                        │   Actions   │
                                        └─────────────┘
```

### Component Details

#### 1. Vision Encoder

Converts images to token embeddings:

```python
from transformers import CLIPVisionModel, CLIPProcessor

class VisionEncoder:
    def __init__(self):
        # Use CLIP vision encoder (pre-trained on image-text pairs)
        self.model = CLIPVisionModel.from_pretrained("openai/clip-vit-base-patch32")
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

    def encode(self, image):
        """
        Args:
            image: PIL Image or numpy array (H, W, 3)
        Returns:
            vision_tokens: (N_patches, hidden_dim)
        """
        # Preprocess image
        inputs = self.processor(images=image, return_tensors="pt")

        # Extract features
        outputs = self.model(**inputs)
        vision_tokens = outputs.last_hidden_state  # (1, N_patches, 768)

        return vision_tokens.squeeze(0)  # (N_patches, 768)
```

**Key architectures:**
- **ViT (Vision Transformer)**: Patch-based image encoding
- **CLIP**: Vision-language contrastive learning
- **DinoV2**: Self-supervised vision features
- **R3M**: Robot-specific vision representations

#### 2. Language Encoder

Encodes text instructions:

```python
from transformers import T5Tokenizer, T5EncoderModel

class LanguageEncoder:
    def __init__(self):
        self.tokenizer = T5Tokenizer.from_pretrained("google/flan-t5-base")
        self.model = T5EncoderModel.from_pretrained("google/flan-t5-base")

    def encode(self, instruction):
        """
        Args:
            instruction: str, e.g., "pick up the red cube"
        Returns:
            language_tokens: (seq_len, hidden_dim)
        """
        # Tokenize
        inputs = self.tokenizer(instruction, return_tensors="pt")

        # Encode
        outputs = self.model(**inputs)
        language_tokens = outputs.last_hidden_state  # (1, seq_len, 768)

        return language_tokens.squeeze(0)  # (seq_len, 768)
```

**Common encoders:**
- **T5**: Text-to-text transformer
- **BERT**: Bidirectional language model
- **LLaMA/GPT**: Large language models
- **CLIP Text**: Aligned with vision

#### 3. Transformer Core

Fuses vision and language, outputs action representations:

```python
import torch
import torch.nn as nn

class VLATransformer(nn.Module):
    def __init__(self, hidden_dim=768, num_layers=6, num_heads=12):
        super().__init__()

        # Multimodal transformer
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=num_heads,
            dim_feedforward=hidden_dim * 4,
            batch_first=True
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        # Modality embeddings (distinguish vision vs language tokens)
        self.vision_embedding = nn.Parameter(torch.randn(1, 1, hidden_dim))
        self.language_embedding = nn.Parameter(torch.randn(1, 1, hidden_dim))

    def forward(self, vision_tokens, language_tokens):
        """
        Args:
            vision_tokens: (batch, N_patches, hidden_dim)
            language_tokens: (batch, seq_len, hidden_dim)
        Returns:
            fused_features: (batch, N_patches + seq_len, hidden_dim)
        """
        batch_size = vision_tokens.shape[0]

        # Add modality embeddings
        vision_tokens = vision_tokens + self.vision_embedding
        language_tokens = language_tokens + self.language_embedding

        # Concatenate vision and language tokens
        combined_tokens = torch.cat([vision_tokens, language_tokens], dim=1)

        # Transform
        fused_features = self.transformer(combined_tokens)

        return fused_features
```

#### 4. Action Decoder

Predicts robot actions from fused features:

```python
class ActionDecoder(nn.Module):
    def __init__(self, hidden_dim=768, action_dim=7, action_horizon=10):
        super().__init__()

        self.action_dim = action_dim
        self.action_horizon = action_horizon  # Predict multiple timesteps

        # Action query tokens (learnable)
        self.action_queries = nn.Parameter(
            torch.randn(1, action_horizon, hidden_dim)
        )

        # Cross-attention: action queries attend to fused features
        self.cross_attention = nn.MultiheadAttention(
            embed_dim=hidden_dim,
            num_heads=12,
            batch_first=True
        )

        # Action head
        self.action_head = nn.Linear(hidden_dim, action_dim)

    def forward(self, fused_features):
        """
        Args:
            fused_features: (batch, seq_len, hidden_dim)
        Returns:
            actions: (batch, action_horizon, action_dim)
        """
        batch_size = fused_features.shape[0]

        # Expand action queries for batch
        action_queries = self.action_queries.expand(batch_size, -1, -1)

        # Cross-attend to fused features
        action_features, _ = self.cross_attention(
            query=action_queries,
            key=fused_features,
            value=fused_features
        )

        # Predict actions
        actions = self.action_head(action_features)  # (batch, horizon, action_dim)

        return actions
```

### Complete VLA Model

```python
class VLAModel(nn.Module):
    """Complete Vision-Language-Action model."""

    def __init__(self, action_dim=7, action_horizon=10):
        super().__init__()

        self.vision_encoder = VisionEncoder()
        self.language_encoder = LanguageEncoder()
        self.transformer = VLATransformer(hidden_dim=768)
        self.action_decoder = ActionDecoder(hidden_dim=768, action_dim=action_dim, action_horizon=action_horizon)

    def forward(self, image, instruction):
        """
        Args:
            image: (batch, H, W, 3) or PIL Image
            instruction: str or List[str]
        Returns:
            actions: (batch, action_horizon, action_dim)
        """
        # Encode vision
        vision_tokens = self.vision_encoder.encode(image)  # (N_patches, 768)
        vision_tokens = vision_tokens.unsqueeze(0)  # (1, N_patches, 768)

        # Encode language
        language_tokens = self.language_encoder.encode(instruction)  # (seq_len, 768)
        language_tokens = language_tokens.unsqueeze(0)  # (1, seq_len, 768)

        # Fuse modalities
        fused_features = self.transformer(vision_tokens, language_tokens)

        # Decode actions
        actions = self.action_decoder(fused_features)

        return actions
```

## Training VLA Models

### Dataset Requirements

**Data format:**
```python
{
    "image": PIL.Image (H, W, 3),
    "instruction": str,
    "actions": np.array (action_horizon, action_dim),
    "success": bool
}
```

**Example trajectory:**
```python
trajectory = {
    "task": "pick up the red block",
    "frames": [
        {
            "image": image_0,  # 640x480 RGB
            "instruction": "pick up the red block",
            "action": [0.1, 0.0, -0.05, 0.0, 0.0, 0.0, 0.0],  # (x, y, z, roll, pitch, yaw, gripper)
            "reward": 0.0
        },
        {
            "image": image_1,
            "instruction": "pick up the red block",
            "action": [0.15, 0.0, -0.1, 0.0, 0.0, 0.0, 0.0],
            "reward": 0.0
        },
        # ... more frames
        {
            "image": image_final,
            "instruction": "pick up the red block",
            "action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],  # Close gripper
            "reward": 1.0  # Success!
        }
    ]
}
```

### Training Loop

```python
import torch
from torch.optim import AdamW
from torch.utils.data import DataLoader

def train_vla(model, dataloader, num_epochs=10):
    """Train VLA model with behavior cloning."""

    optimizer = AdamW(model.parameters(), lr=1e-4)
    criterion = nn.MSELoss()  # Action prediction loss

    model.train()

    for epoch in range(num_epochs):
        total_loss = 0.0

        for batch in dataloader:
            images = batch["image"]  # (batch, H, W, 3)
            instructions = batch["instruction"]  # List[str]
            target_actions = batch["actions"]  # (batch, horizon, action_dim)

            # Forward pass
            predicted_actions = model(images, instructions)

            # Compute loss
            loss = criterion(predicted_actions, target_actions)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.4f}")

    return model
```

### Data Collection Strategies

#### 1. Teleoperation
Human demonstrates tasks via VR/keyboard:
```python
def collect_teleop_data(robot, num_episodes=100):
    """Collect data via human teleoperation."""
    dataset = []

    for episode in range(num_episodes):
        # Show random task
        task = sample_task()  # e.g., "pick red cube"
        print(f"Task: {task}")

        # Human controls robot
        trajectory = robot.run_teleoperation(task)

        # Save trajectory
        dataset.append({
            "task": task,
            "trajectory": trajectory
        })

    return dataset
```

#### 2. Simulation (Isaac Sim)
Generate data automatically:
```python
def collect_sim_data(env, policy, num_episodes=10000):
    """Collect data in simulation with domain randomization."""
    dataset = []

    for episode in range(num_episodes):
        obs = env.reset()
        task = env.sample_task()

        trajectory = []
        done = False

        while not done:
            # Use pre-trained policy or scripted behavior
            action = policy(obs, task)

            # Step environment
            next_obs, reward, done, info = env.step(action)

            # Record frame
            trajectory.append({
                "image": obs["image"],
                "instruction": task,
                "action": action
            })

            obs = next_obs

        if info["success"]:
            dataset.append(trajectory)

    return dataset
```

#### 3. Co-Training (Sim + Real)
Mix simulated and real data:
```python
# 90% sim, 10% real
dataset = combine_datasets(
    sim_data=collect_sim_data(sim_env, num_episodes=9000),
    real_data=collect_teleop_data(real_robot, num_episodes=1000),
    real_weight=10.0  # Upweight real data
)
```

## Using Pre-Trained VLA Models

### OpenVLA (7B Parameters)

```python
from transformers import AutoModel, AutoProcessor

# Load OpenVLA
model = AutoModel.from_pretrained("openvla/openvla-7b")
processor = AutoProcessor.from_pretrained("openvla/openvla-7b")

# Inference
image = load_image("scene.jpg")
instruction = "pick up the blue mug"

inputs = processor(images=image, text=instruction, return_tensors="pt")
outputs = model(**inputs)
actions = outputs.actions  # (1, action_horizon, 7)

# Execute action
robot.execute(actions[0, 0])  # First timestep action
```

### RT-2 (via Hugging Face)

```python
# RT-2 is not publicly released, but similar models:
from transformers import RT2Model  # Hypothetical

model = RT2Model.from_pretrained("google/rt-2-base")

# Run inference
action = model.predict(image=image, instruction=instruction)
robot.move_to(action)
```

## Integrating VLA with ROS 2

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch

class VLANode(Node):
    """ROS 2 node for VLA model inference."""

    def __init__(self):
        super().__init__('vla_node')

        # Load VLA model
        self.model = VLAModel.from_pretrained("path/to/model")
        self.model.eval()

        # CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, '/voice/instruction', self.instruction_callback, 10
        )

        # Publisher
        self.action_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.latest_image = None
        self.latest_instruction = None

        # Timer for inference
        self.create_timer(0.1, self.inference_callback)  # 10 Hz

    def image_callback(self, msg):
        """Store latest image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def instruction_callback(self, msg):
        """Store latest instruction."""
        self.latest_instruction = msg.data
        self.get_logger().info(f'Received instruction: {msg.data}')

    def inference_callback(self):
        """Run VLA inference and publish actions."""
        if self.latest_image is None or self.latest_instruction is None:
            return

        # Run VLA model
        with torch.no_grad():
            actions = self.model(self.latest_image, self.latest_instruction)

        # Convert to Twist message (simplified)
        action = actions[0, 0].cpu().numpy()  # First action
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[5])

        # Publish
        self.action_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VLANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Week 11 Hands-On Exercise

**Task**: Fine-tune a pre-trained vision encoder for robot manipulation

**Requirements:**
1. Download RT-1 or OpenVLA demo dataset (100 trajectories)
2. Fine-tune CLIP vision encoder on robot images
3. Implement simple action prediction head (MLP)
4. Train for 10 epochs
5. Evaluate action prediction accuracy
6. Visualize predicted vs ground-truth actions

**Bonus:**
- Add language conditioning
- Test on Isaac Sim environment
- Implement action chunking (predict 10 steps ahead)

## Resources

- [RT-1 Paper](https://arxiv.org/abs/2212.06817) - Robotics Transformer for Real-World Control
- [RT-2 Paper](https://arxiv.org/abs/2307.15818) - Vision-Language-Action Models
- [OpenVLA](https://openvla.github.io/) - Open-source 7B VLA model
- [RoboFlamingo](https://roboflamingo.github.io/) - Open-weights VLA
- [PaLM-E Paper](https://arxiv.org/abs/2303.03378) - Embodied Multimodal Language Model

## Next Steps

Excellent work! You now understand VLA architecture and can work with pre-trained models.

Next week: [Week 12: Advanced VLA Applications](week-12.md)

We'll explore action chunking, multi-task learning, and deploying VLA models on real robots!
