# Week 10: Sim-to-Real Transfer & Chapter 3 Project

## Overview

This final week of Chapter 3 addresses the critical challenge of sim-to-real transfer: making policies and models trained in simulation work on real robots. You'll learn proven techniques, implement transfer strategies, and complete a comprehensive Isaac Sim project.

## Learning Objectives

By the end of this week, you will be able to:

- Understand the causes of the sim-to-real gap
- Apply domain randomization techniques systematically
- Implement system identification for physics calibration
- Use sim-to-real transfer best practices
- Evaluate transfer success quantitatively
- Complete the Chapter 3 assessment project
- Deploy simulated policies to real hardware (conceptually)

## The Sim-to-Real Gap

### What is the Sim-to-Real Gap?

**Definition**: The performance degradation when transferring policies/models from simulation to the real world.

**Example:**
- **In simulation**: Robot grasps objects 95% of the time
- **On real robot**: Robot grasps objects 40% of the time
- **Gap**: 55% performance loss

### Root Causes

#### 1. Physics Discrepancies

| Property | Simulation | Reality |
|----------|------------|---------|
| **Friction** | Constant, simplified | Variable, complex |
| **Contact** | Penetration-based | Deformation, slip |
| **Dynamics** | Deterministic | Stochastic |
| **Delays** | None | Motor lag, sensor latency |
| **Noise** | Gaussian (if added) | Non-Gaussian, time-correlated |

#### 2. Visual Discrepancies

- **Rendering**: Perfect raytracing vs real camera noise/blur
- **Lighting**: Controlled vs variable ambient light
- **Textures**: Clean 3D models vs worn/dirty real objects
- **Occlusion**: Perfect vs partial sensor coverage

#### 3. Modeling Errors

- **Simplified geometry**: CAD models vs manufactured tolerances
- **Mass/inertia errors**: Estimated vs actual properties
- **Sensor models**: Idealized vs real sensor characteristics
- **Actuation**: Perfect motors vs backlash/compliance

## Bridging the Gap: Proven Techniques

### 1. Domain Randomization (DR)

**Idea**: Randomize simulation parameters so the real world is just another variation.

**Parameters to randomize:**

```python
import numpy as np

class DomainRandomizer:
    """Comprehensive domain randomization."""

    def __init__(self):
        self.params = {}

    def randomize_physics(self):
        """Randomize physics properties."""
        # Friction coefficients
        self.params['friction'] = np.random.uniform(0.3, 1.5)

        # Mass (±20% of nominal)
        self.params['mass_scale'] = np.random.uniform(0.8, 1.2)

        # Joint damping
        self.params['joint_damping'] = np.random.uniform(0.01, 0.5)

        # Motor strength (±10%)
        self.params['motor_scale'] = np.random.uniform(0.9, 1.1)

        # Action delay (0-50ms)
        self.params['action_delay'] = np.random.uniform(0, 0.05)

    def randomize_vision(self):
        """Randomize visual appearance."""
        # Lighting
        self.params['light_intensity'] = np.random.uniform(1000, 50000)
        self.params['light_color'] = np.random.random(3)  # RGB

        # Camera
        self.params['exposure'] = np.random.uniform(0.5, 2.0)
        self.params['gamma'] = np.random.uniform(0.8, 1.2)
        self.params['noise_std'] = np.random.uniform(0, 0.02)  # Gaussian noise

        # Object appearance
        self.params['object_color'] = np.random.random(3)
        self.params['object_texture'] = np.random.choice([
            "smooth", "rough", "metallic", "matte"
        ])

    def randomize_geometry(self):
        """Randomize sizes and positions."""
        # Object size (±5%)
        self.params['size_scale'] = np.random.uniform(0.95, 1.05)

        # Spawn position noise (±2cm)
        self.params['position_noise'] = np.random.uniform(-0.02, 0.02, size=3)

        # Orientation noise (±5 degrees)
        self.params['rotation_noise'] = np.random.uniform(-0.087, 0.087, size=3)

    def randomize_all(self):
        """Apply all randomizations."""
        self.randomize_physics()
        self.randomize_vision()
        self.randomize_geometry()
        return self.params
```

**Training with DR:**

```python
def train_with_dr(env, num_episodes=10000):
    """Train policy with domain randomization."""
    randomizer = DomainRandomizer()

    for episode in range(num_episodes):
        # Randomize domain for this episode
        params = randomizer.randomize_all()
        env.apply_randomization(params)

        # Run episode
        obs = env.reset()
        done = False

        while not done:
            action = policy(obs)
            obs, reward, done, info = env.step(action)

        if episode % 100 == 0:
            print(f"Episode {episode}: Avg reward = {avg_reward}")
```

### 2. System Identification

**Goal**: Measure real robot properties and match simulation.

**Step 1: Identify Friction**

```python
def identify_friction(robot):
    """
    Apply constant force, measure terminal velocity.
    friction_coef = force / (mass * g)
    """
    forces = [0.5, 1.0, 1.5, 2.0]  # Newtons
    velocities = []

    for force in forces:
        robot.apply_force(force)
        time.sleep(2.0)  # Wait to reach terminal velocity
        vel = robot.get_velocity()
        velocities.append(vel)

    # Linear regression: F = μ * m * g + friction_loss
    # Simplified: μ ≈ F / (m * g)
    mass = 5.0  # kg (known)
    g = 9.81
    friction_coef = np.mean(forces) / (mass * g)

    return friction_coef
```

**Step 2: Update Simulation**

```xml
<!-- Update URDF/USD with identified parameters -->
<gazebo reference="link">
  <mu1>0.68</mu1>  <!-- Identified friction -->
  <mu2>0.68</mu2>
</gazebo>
```

### 3. Privileged Learning + Adaptation

**Idea**: Train with perfect sim information, then adapt at deployment.

```python
class PrivilegedPolicy:
    """Policy that uses privileged information during training."""

    def __init__(self):
        # Student policy (deployed)
        self.student = StudentPolicy(obs_dim=10, action_dim=4)

        # Teacher policy (training only, has privileged info)
        self.teacher = TeacherPolicy(obs_dim=10, priv_dim=20, action_dim=4)

    def train_step(self, obs, privileged_info, true_action):
        """Train both policies."""
        # Teacher uses privileged info (true object mass, friction, etc.)
        teacher_action = self.teacher(obs, privileged_info)

        # Student tries to match teacher without privileged info
        student_action = self.student(obs)

        # Losses
        teacher_loss = mse_loss(teacher_action, true_action)
        distillation_loss = mse_loss(student_action, teacher_action)

        total_loss = teacher_loss + distillation_loss
        return total_loss

    def deploy(self, obs):
        """At deployment, only use student."""
        return self.student(obs)
```

**Privileged information examples:**
- True object mass, friction coefficients
- Ground-truth object poses (vs noisy perception)
- Future trajectory (for prediction tasks)
- Hidden state (joint forces, contact points)

### 4. Fine-Tuning on Real Data

**Strategy**: Train in sim, fine-tune with small real dataset.

```python
# Phase 1: Pre-train in simulation (millions of samples)
policy = train_in_simulation(num_steps=10_000_000)

# Phase 2: Collect real data (hundreds of samples)
real_data = collect_real_robot_data(num_episodes=100)

# Phase 3: Fine-tune on real data
policy = fine_tune(policy, real_data, num_epochs=50, lr=1e-5)
```

**Best practices:**
- Use low learning rate to avoid catastrophic forgetting
- Keep 90% of training data from simulation
- Focus real data on failure modes

### 5. Residual Learning

**Idea**: Learn correction on top of sim policy.

```python
class ResidualPolicy:
    """Sim policy + learned residual."""

    def __init__(self, sim_policy):
        self.sim_policy = sim_policy  # Frozen
        self.residual_network = ResidualNet(obs_dim=10, action_dim=4)

    def forward(self, obs):
        # Get sim policy action
        sim_action = self.sim_policy(obs)

        # Compute residual (correction)
        residual = self.residual_network(obs)

        # Final action = sim + residual
        action = sim_action + residual
        return action
```

**Training:**
- Train residual network on real robot
- Keeps sim policy knowledge, only learns corrections

## Quantitative Transfer Evaluation

### Metrics

**1. Success Rate**
```
Success Rate = (Successful Trials / Total Trials) × 100%
```

**2. Sim-to-Real Performance Ratio**
```
Transfer Ratio = Real Performance / Sim Performance
```
- Ratio = 1.0: Perfect transfer
- Ratio < 0.7: Poor transfer (needs improvement)
- Ratio > 0.9: Excellent transfer

**3. Sample Efficiency**
```
Samples Needed = Real samples to reach 90% sim performance
```

**4. Task-Specific Metrics**
- **Grasping**: Grasp success rate, grasp stability
- **Navigation**: Goal-reaching success, collision rate
- **Manipulation**: Task completion time, precision

### Evaluation Protocol

```python
def evaluate_transfer(policy, real_env, num_trials=100):
    """Evaluate sim-to-real transfer."""
    successes = 0
    completion_times = []
    failures = {"collision": 0, "timeout": 0, "grasp_fail": 0}

    for trial in range(num_trials):
        obs = real_env.reset()
        done = False
        steps = 0

        while not done and steps < max_steps:
            action = policy(obs)
            obs, reward, done, info = real_env.step(action)
            steps += 1

        # Record outcome
        if info["success"]:
            successes += 1
            completion_times.append(steps)
        else:
            failure_type = info["failure_reason"]
            failures[failure_type] += 1

    # Compute metrics
    success_rate = successes / num_trials
    avg_time = np.mean(completion_times) if completion_times else None

    return {
        "success_rate": success_rate,
        "avg_completion_time": avg_time,
        "failure_breakdown": failures
    }
```

## Case Study: Grasping Transfer

### Simulation Setup

```python
# Train grasping policy in Isaac Sim
env = GraspingEnv(
    num_envs=2048,
    domain_randomization=True,
    object_types=["cube", "cylinder", "sphere", "irregular"],
    object_textures=textures_library,  # 100+ textures
    lighting_range=(5000, 50000),
    friction_range=(0.3, 1.5)
)

# Train with PPO
model = PPO("MultiInputPolicy", env, learning_rate=3e-4)
model.learn(total_timesteps=5_000_000)
```

### Real Robot Deployment

```python
# Load sim-trained policy
policy = load_policy("grasp_policy_sim.pth")

# Real robot environment
real_env = RealRobotEnv(
    camera_topic="/camera/rgb",
    robot_ip="192.168.1.10"
)

# Evaluate
results = evaluate_transfer(policy, real_env, num_trials=50)
print(f"Sim-to-Real Success Rate: {results['success_rate']:.1%}")
```

**Typical results:**
- No DR: 30-40% success
- With DR: 70-85% success
- DR + Fine-tuning: 85-95% success

## Chapter 3 Assessment Project

**Task**: Build a complete Isaac Sim pipeline for a manipulation task

### Project Requirements (100 points)

#### Part 1: Scene Setup (15 points)
- Create warehouse/factory environment
- Add obstacles, varying lighting
- Include 3+ object types for manipulation
- Custom USD models (bonus: import from CAD)

#### Part 2: Synthetic Data Generation (25 points)
- Generate 5000+ images with annotations
- Implement comprehensive domain randomization:
  - Lighting (3+ sources, varying intensity/color)
  - Object poses, sizes, textures
  - Camera parameters, noise
- Export in COCO or custom format
- Split: 80% train, 10% val, 10% test

#### Part 3: Vision Model Training (25 points)
- Train object detection OR segmentation model
- Use synthetic data only
- Report metrics on validation set:
  - mAP@0.5 for detection
  - IoU for segmentation
- Visualize predictions on test images

#### Part 4: RL Policy Training (25 points)
- Define manipulation task (pick-and-place, reaching, etc.)
- Implement environment with domain randomization
- Train policy with RL algorithm (PPO/SAC/etc.)
- Demonstrate successful task execution in sim
- Report training curves, success rate

#### Part 5: Documentation (10 points)
- README with setup instructions
- Architecture diagram (scene, sensors, algorithms)
- Training/evaluation reports
- Demo video (5 minutes max showing full pipeline)

### Deliverables

1. **GitHub Repository**:
   - All source code
   - Dataset generation scripts
   - Training scripts
   - Trained models/policies

2. **Dataset**:
   - Link to generated dataset (cloud storage OK)
   - 100 sample images in repository

3. **Report** (PDF, 3-5 pages):
   - Methodology
   - Domain randomization strategies
   - Training metrics and curves
   - Challenges and solutions

4. **Demo Video**:
   - Scene walkthrough
   - Dataset generation process
   - Model/policy in action
   - Quantitative results

### Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Scene Quality** | 15 | Realistic, diverse, well-lit |
| **Dataset Quality** | 15 | Large, diverse, properly annotated |
| **DR Implementation** | 10 | Comprehensive randomization |
| **Model Performance** | 15 | Meets accuracy thresholds |
| **RL Policy** | 15 | Task success > 80% in sim |
| **Code Quality** | 10 | Clean, documented, reproducible |
| **Documentation** | 10 | Clear, complete, well-written |
| **Creativity** | 10 | Novel approach or extra features |

**Bonus Opportunities** (+20 points max):
- Deploy on real robot (+15)
- Multi-task learning (+10)
- Custom physics simulation (+10)
- SLAM integration (+10)

### Example Projects

**Example 1: Bin Picking**
- Task: Pick parts from cluttered bin
- Objects: Screws, nuts, washers (varying sizes)
- Vision: YOLOv8 for detection
- Policy: PPO for pick-and-place

**Example 2: Quality Inspection**
- Task: Detect defects on manufactured parts
- Dataset: 10K images with defect annotations
- Model: Semantic segmentation (U-Net)
- Deployment: ROS 2 node for real-time inference

**Example 3: Warehouse Navigation**
- Task: Navigate to shelf, retrieve item
- Environment: 50x50m warehouse with aisles
- Vision: Depth camera for obstacle avoidance
- Policy: SAC for navigation + grasping

## Week 10 Hands-On Exercise

**Task**: Implement and compare DR strategies

1. Train baseline policy **without** DR
2. Train policy **with** lighting DR only
3. Train policy **with** full DR (lighting + physics + vision)
4. Evaluate all three in varied test environments
5. Report performance comparison

**Expected outcome**: Full DR policy should be most robust to environment changes.

## Resources

- [Sim-to-Real Transfer Survey](https://arxiv.org/abs/2009.13303)
- [Domain Randomization for Transferring Deep Neural Networks](https://arxiv.org/abs/1703.06907)
- [Learning Dexterous In-Hand Manipulation](https://arxiv.org/abs/1808.00177) - OpenAI's Rubik's Cube
- [NVIDIA Isaac Sim Examples](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)
- [System Identification Techniques](https://stanford.edu/class/ee363/sysid.pdf)

## Chapter 3 Summary

Congratulations on completing Chapter 3! You've learned:

✅ NVIDIA Isaac Sim setup and navigation
✅ USD scene creation and robot import
✅ ROS 2 integration for sensor data
✅ Synthetic data generation with Replicator
✅ Domain randomization strategies
✅ Massively parallel RL with Isaac Gym
✅ Sim-to-real transfer techniques
✅ Complete ML pipeline from sim to deployment

## Next Steps

**What's next:**
- Complete Chapter 3 assessment project
- Review Isaac Sim concepts as needed
- Prepare for Chapter 4: [Vision-Language-Action Models & Capstone](../04-vla/index.md)

Chapter 4 will bring everything together: multimodal AI, end-to-end robot systems, and your final capstone project!
