# Week 9: Synthetic Data Generation & Isaac Gym

## Overview

This week explores two powerful Isaac Sim capabilities: **Synthetic Data Generation** (SDG) for training vision models and **Isaac Gym** for massively parallel reinforcement learning. You'll learn how to create realistic training datasets and train robot policies entirely in simulation.

## Learning Objectives

By the end of this week, you will be able to:

- Understand the value of synthetic data for AI/ML
- Use Isaac Sim Replicator for dataset generation
- Create domain-randomized training data
- Generate annotated datasets (bounding boxes, segmentation masks)
- Set up Isaac Gym for reinforcement learning
- Train a simple RL policy (reaching, grasping)
- Evaluate sim-to-real transfer readiness

## Why Synthetic Data?

### The Data Problem in Robotics

**Traditional approach:**
1. Build physical robot ($10K-$1M)
2. Collect data manually (weeks/months)
3. Label data manually (expensive, error-prone)
4. Train model
5. Repeat when model fails in new scenarios

**Synthetic data approach:**
1. Create simulation (days)
2. Generate millions of samples automatically (hours)
3. Perfect labels automatically (free)
4. Train model
5. Add domain randomization for new scenarios

### Benefits of Synthetic Data

| Aspect | Real Data | Synthetic Data |
|--------|-----------|----------------|
| **Cost** | $$$$ (hardware, labor) | $ (compute only) |
| **Speed** | Slow (physical collection) | Fast (parallel generation) |
| **Scale** | 1000s of images | Millions of images |
| **Labels** | Manual ($0.10-$1/image) | Automatic (free, perfect) |
| **Diversity** | Limited by physical setup | Unlimited (domain randomization) |
| **Safety** | Risk of damage | Risk-free |
| **Edge cases** | Hard to capture | Easy to create |

### Challenges & Solutions

**Challenge 1: Sim-to-Real Gap**
- Simulated images look "fake" compared to reality
- **Solution**: Domain randomization (lighting, textures, camera params)

**Challenge 2: Physics Mismatch**
- Simulated physics differs from real-world
- **Solution**: System identification, fine-tuning on real data

**Challenge 3: Overfitting to Simulation**
- Model works in sim but fails on real robot
- **Solution**: Diverse randomization, sim-to-real transfer techniques

## Isaac Sim Replicator

**Replicator** is Isaac Sim's synthetic data generation framework.

### Key Capabilities

- **Randomization**: Materials, lighting, camera params, object poses
- **Annotations**: Bounding boxes, segmentation, depth, normals
- **Scalability**: Generate 1000s of images in parallel
- **Formats**: COCO, KITTI, Custom JSON

### Replicator Workflow

```
1. Create base scene
   ↓
2. Define randomizers (lighting, materials, poses)
   ↓
3. Define writers (save annotations)
   ↓
4. Run generation loop
   ↓
5. Export dataset
```

## Creating a Synthetic Dataset

### Example: Object Detection Dataset

**Goal**: Train YOLOv8 to detect boxes on a table

#### Step 1: Create Base Scene

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})  # No GUI for speed

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims import GeometryPrim
from omni.replicator.core import randomizer, Writer
import omni.replicator.core as rep
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Create table
table = world.scene.add(
    VisualCuboid(
        prim_path="/World/Table",
        name="table",
        position=np.array([0, 0, 0.5]),
        size=np.array([1.0, 1.0, 0.05]),
        color=np.array([0.5, 0.3, 0.1])  # Brown
    )
)

# Create camera looking at table
camera = rep.create.camera(
    position=(0, -2, 1.5),
    look_at=(0, 0, 0.5)
)

# Create light
light = rep.create.light(
    light_type="Sphere",
    intensity=30000,
    position=(2, 2, 3),
    scale=0.5
)
```

#### Step 2: Define Randomizers

```python
import omni.replicator.core as rep

# Randomize object positions
def randomize_objects():
    """Place 1-5 boxes randomly on table."""
    num_objects = np.random.randint(1, 6)

    for i in range(num_objects):
        # Random position on table
        x = np.random.uniform(-0.4, 0.4)
        y = np.random.uniform(-0.4, 0.4)
        z = 0.525  # Just above table

        # Random size
        size = np.random.uniform(0.05, 0.15)

        # Random color
        color = np.random.random(3)

        # Create box
        box = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Box_{i}",
                name=f"box_{i}",
                position=np.array([x, y, z]),
                size=np.array([size, size, size]),
                color=color
            )
        )

# Randomize lighting
def randomize_lighting():
    """Vary light intensity and position."""
    intensity = np.random.uniform(20000, 40000)
    x = np.random.uniform(-3, 3)
    y = np.random.uniform(-3, 3)
    z = np.random.uniform(2, 4)

    # Update light (use replicator API)
    with rep.new_layer():
        light = rep.get.prims(path_pattern="/World/Lights/*")
        with light:
            rep.modify.pose(position=(x, y, z))
            rep.modify.attribute("intensity", intensity)

# Randomize camera
def randomize_camera():
    """Vary camera position around table."""
    # Spherical coordinates
    radius = np.random.uniform(1.5, 2.5)
    theta = np.random.uniform(-np.pi/4, np.pi/4)  # ±45 degrees
    phi = np.random.uniform(np.pi/6, np.pi/3)     # 30-60 degrees elevation

    x = radius * np.cos(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.cos(phi)
    z = radius * np.sin(phi)

    with rep.new_layer():
        camera = rep.get.prims(path_pattern="/World/Camera")
        with camera:
            rep.modify.pose(position=(x, y, z), look_at=(0, 0, 0.5))
```

#### Step 3: Register Annotators

```python
# Enable annotations
rp = rep.create.render_product(camera, (640, 480))

# RGB images
rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach(rp)

# Bounding boxes (2D)
bbox_2d_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
bbox_2d_annot.attach(rp)

# Semantic segmentation
semantic_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
semantic_annot.attach(rp)

# Depth
depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
depth_annot.attach(rp)
```

#### Step 4: Custom Writer (COCO Format)

```python
import omni.replicator.core as rep
import json
import os
from PIL import Image

class COCOWriter(rep.Writer):
    """Export dataset in COCO format."""

    def __init__(self, output_dir):
        super().__init__()
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(f"{output_dir}/images", exist_ok=True)

        self.coco_data = {
            "images": [],
            "annotations": [],
            "categories": [{"id": 1, "name": "box"}]
        }
        self.image_id = 0
        self.annot_id = 0

    def write(self, data):
        """Called for each frame."""
        # Save RGB image
        rgb = data["rgb"]
        img_filename = f"image_{self.image_id:06d}.png"
        img_path = f"{self.output_dir}/images/{img_filename}"
        Image.fromarray(rgb).save(img_path)

        # Add image metadata
        self.coco_data["images"].append({
            "id": self.image_id,
            "file_name": img_filename,
            "width": rgb.shape[1],
            "height": rgb.shape[0]
        })

        # Add bounding box annotations
        bboxes = data["bounding_box_2d_tight"]
        for bbox in bboxes:
            x_min, y_min, x_max, y_max = bbox
            width = x_max - x_min
            height = y_max - y_min

            self.coco_data["annotations"].append({
                "id": self.annot_id,
                "image_id": self.image_id,
                "category_id": 1,
                "bbox": [x_min, y_min, width, height],
                "area": width * height,
                "iscrowd": 0
            })
            self.annot_id += 1

        self.image_id += 1

    def on_final_frame(self):
        """Called after last frame."""
        # Save COCO JSON
        with open(f"{self.output_dir}/annotations.json", "w") as f:
            json.dump(self.coco_data, f, indent=2)

        print(f"Dataset saved to {self.output_dir}")
        print(f"Total images: {self.image_id}")
        print(f"Total annotations: {self.annot_id}")

# Register writer
writer = COCOWriter(output_dir="./dataset_boxes")
writer.attach(rp)
```

#### Step 5: Run Generation

```python
# Generation loop
num_frames = 1000

world.reset()

for i in range(num_frames):
    # Randomize scene
    randomize_objects()
    randomize_lighting()
    randomize_camera()

    # Step physics (let objects settle)
    for _ in range(10):
        world.step(render=False)

    # Capture frame
    world.step(render=True)

    # Trigger replicator write
    rep.orchestrator.step()

    if i % 100 == 0:
        print(f"Generated {i}/{num_frames} frames")

# Finalize
writer.on_final_frame()
simulation_app.close()
```

**Result**: 1000 images with COCO annotations ready for YOLO training!

## Domain Randomization Best Practices

### 1. Lighting Randomization

```python
# Multiple lights with random colors
for i in range(3):
    color = np.random.random(3)
    intensity = np.random.uniform(10000, 50000)
    position = np.random.uniform(-5, 5, size=3)

    light = rep.create.light(
        light_type="Sphere",
        intensity=intensity,
        color=color,
        position=position
    )
```

### 2. Texture Randomization

```python
# Apply random materials to objects
materials = [
    "omni://localhost/NVIDIA/Materials/vMaterials_2/Ground/textures/aggregate_exposed_diff.jpg",
    "omni://localhost/NVIDIA/Materials/vMaterials_2/Wood/textures/wood_cherry_diff.jpg",
    # Add more material paths
]

def randomize_materials():
    boxes = rep.get.prims(semantics=[("class", "box")])
    with boxes:
        rep.randomizer.materials(materials)
```

### 3. Camera Randomization

```python
# Simulate real camera noise
with camera:
    # Motion blur
    rep.modify.attribute("motion_blur:enable", True)
    rep.modify.attribute("motion_blur:intensity", np.random.uniform(0, 0.5))

    # Exposure
    rep.modify.attribute("exposure", np.random.uniform(0.5, 2.0))

    # Focal length (FOV variation)
    rep.modify.attribute("focalLength", np.random.uniform(18, 55))
```

### 4. Background Randomization

```python
# Use random HDRI backgrounds
hdris = [
    "omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCG_com_WarehouseInterior2.hdr",
    "omniverse://localhost/NVIDIA/Assets/Skies/Outdoor/kloppenheim_06_4k.hdr",
]

with rep.new_layer():
    dome_light = rep.create.light(light_type="Dome")
    with dome_light:
        rep.randomizer.texture(hdris)
```

## Isaac Gym: Massively Parallel RL

**Isaac Gym** enables training 1000s of robot policies in parallel on a single GPU.

### Key Concepts

- **Vectorized environments**: Run 1000+ instances simultaneously
- **GPU physics**: All simulation on GPU (no CPU bottleneck)
- **GPU tensors**: Observations/actions stay on GPU (no CPU↔GPU transfer)
- **Fast**: Train policies in minutes instead of days

### Isaac Gym vs Traditional RL

| Metric | Traditional (CPU) | Isaac Gym (GPU) |
|--------|-------------------|-----------------|
| **Parallel envs** | 8-16 | 1024-8192 |
| **Timesteps/sec** | 1000-5000 | 100K-1M |
| **Training time (Reach)** | 24 hours | 5 minutes |
| **Hardware** | Multi-core CPU | Single RTX GPU |

### Simple Reach Task

**Goal**: Train robot arm to reach random target positions

#### Environment Setup

```python
from omni.isaac.gym import VecEnvBase
import torch
import numpy as np

class ReachEnv(VecEnvBase):
    """Simple reaching task."""

    def __init__(self, num_envs=1024, device="cuda:0"):
        self.num_envs = num_envs
        self.device = device

        # Observation: [joint positions (7), target position (3)]
        self.num_obs = 10

        # Action: target joint positions (7)
        self.num_actions = 7

        super().__init__(num_envs=num_envs)

        # Initialize targets
        self.targets = torch.zeros((num_envs, 3), device=device)

    def reset(self):
        """Reset all environments."""
        # Randomize target positions
        self.targets = torch.rand((self.num_envs, 3), device=self.device)
        self.targets[:, 0] = self.targets[:, 0] * 0.6 + 0.2  # X: 0.2-0.8
        self.targets[:, 1] = (self.targets[:, 1] - 0.5) * 0.6 # Y: -0.3-0.3
        self.targets[:, 2] = self.targets[:, 2] * 0.5 + 0.2   # Z: 0.2-0.7

        # Reset robot to home position
        home_joints = torch.tensor([0, -1.0, 0, -2.2, 0, 2.4, 0.8], device=self.device)
        self.joint_positions = home_joints.repeat(self.num_envs, 1)

        return self.get_observations()

    def get_observations(self):
        """Get current observations."""
        # Concatenate joint positions and target
        obs = torch.cat([self.joint_positions, self.targets], dim=1)
        return obs

    def step(self, actions):
        """Apply actions and step simulation."""
        # Actions are target joint positions
        self.joint_positions = actions

        # Get end-effector position (simplified - use forward kinematics in reality)
        ee_pos = self.compute_ee_position(self.joint_positions)

        # Compute reward: negative distance to target
        distance = torch.norm(ee_pos - self.targets, dim=1)
        rewards = -distance

        # Episode done if reached target (distance < 0.05m)
        dones = distance < 0.05

        # Get new observations
        obs = self.get_observations()

        return obs, rewards, dones, {}

    def compute_ee_position(self, joint_positions):
        """Simplified forward kinematics."""
        # In reality, use proper FK
        # Here, just approximate for demonstration
        return torch.rand((self.num_envs, 3), device=self.device)
```

#### Training with PPO

```python
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

# Create environment
env = ReachEnv(num_envs=2048)

# Normalize observations and rewards
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# Create PPO agent
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=16,  # Small for fast updates
    batch_size=4096,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.0,
    verbose=1,
    device="cuda"
)

# Train
model.learn(total_timesteps=1_000_000)

# Save model
model.save("reach_policy")
```

**Training completes in ~5 minutes on RTX 3080!**

### Real Isaac Gym Example (Cartpole)

Isaac Gym includes pre-built tasks:

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/standalone_examples/api/omni.isaac.gym

# Run cartpole example
python cartpole.py
```

Trains 2048 cartpoles in parallel!

## Week 9 Hands-On Project

**Task**: Create a synthetic dataset and train a simple model

**Part 1: Synthetic Dataset (50 points)**
- Scene: Table with 3-6 colored cubes
- Randomization: Lighting (3 sources), cube positions, cube colors
- Generate 2000 images (640x480)
- Annotations: Bounding boxes in COCO format
- Save dataset to disk

**Part 2: Model Training (50 points)**
- Train YOLOv8 or Faster R-CNN on synthetic data
- Evaluate on 200-image validation set
- Report mAP (mean Average Precision)
- Test on real images (if available) to assess sim-to-real gap

**Deliverables:**
- Replicator script for dataset generation
- Training script and logs
- Trained model weights
- Evaluation report with metrics

**Bonus (+20 points):**
- Implement custom domain randomization (distractor objects, camera noise)
- Train RL policy for simple manipulation task

## Resources

- [Replicator Documentation](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)
- [Isaac Gym Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/index.html)
- [Synthetic Data Generation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)
- [Domain Randomization Paper](https://arxiv.org/abs/1703.06907)
- [Isaac Gym Benchmark](https://leggedrobotics.github.io/rl-games/)

## Next Steps

Great work! You now understand synthetic data generation and parallel RL training.

Next week: [Week 10: Sim-to-Real Transfer & Chapter 3 Project](week-10.md)

We'll tackle the sim-to-real gap and complete a comprehensive Isaac Sim project!
