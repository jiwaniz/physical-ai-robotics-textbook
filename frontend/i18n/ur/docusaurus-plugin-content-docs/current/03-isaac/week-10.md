# ہفتہ 10: Sim-to-Real Transfer & Chapter 3 Project

## جائزہ

باب 3 کا یہ آخری ہفتہ sim-to-real transfer کے نازک چیلنج سے نمٹتا ہے: simulation میں trained policies اور models کو حقیقی robots پر کام کرنا۔ آپ ثابت شدہ تکنیکیں سیکھیں گے، transfer حکمت عملی نافذ کریں گے، اور ایک جامع Isaac Sim پروجیکٹ مکمل کریں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- Sim-to-real gap کی وجوہات سمجھیں
- منظم طریقے سے domain randomization تکنیکیں لگائیں
- Physics calibration کے لیے system identification نافذ کریں
- Sim-to-real transfer کے بہترین طریقے استعمال کریں
- Transfer کی کامیابی کا مقداری جائزہ لیں
- باب 3 کا assessment پروجیکٹ مکمل کریں
- Simulated policies کو حقیقی hardware پر deploy کریں (تصوراتی طور پر)

## Sim-to-Real Gap

### Sim-to-Real Gap کیا ہے؟

**تعریف**: Simulation سے حقیقی دنیا میں policies/models منتقل کرتے وقت کارکردگی میں کمی۔

**مثال:**
- **Simulation میں**: Robot 95% وقت اشیاء کو grasp کرتا ہے
- **حقیقی robot پر**: Robot 40% وقت اشیاء کو grasp کرتا ہے
- **Gap**: 55% کارکردگی کا نقصان

### بنیادی وجوہات

#### 1. Physics تضادات

| خاصیت | Simulation | حقیقت |
|----------|------------|---------|
| **Friction** | مستقل، آسان کیا ہوا | متغیر، پیچیدہ |
| **Contact** | Penetration پر مبنی | Deformation، slip |
| **Dynamics** | Deterministic | Stochastic |
| **Delays** | کوئی نہیں | Motor lag، sensor latency |
| **Noise** | Gaussian (اگر شامل ہو) | Non-Gaussian، وقت سے منسلک |

#### 2. بصری تضادات

- **Rendering**: کامل raytracing بمقابلہ حقیقی camera noise/blur
- **Lighting**: کنٹرول شدہ بمقابلہ متغیر محیطی روشنی
- **Textures**: صاف 3D models بمقابلہ گھسے ہوئے/گندے حقیقی اشیاء
- **Occlusion**: کامل بمقابلہ جزوی sensor coverage

#### 3. Modeling کی غلطیاں

- **آسان کی ہوئی geometry**: CAD models بمقابلہ manufactured tolerances
- **Mass/inertia errors**: تخمینہ شدہ بمقابلہ حقیقی خصوصیات
- **Sensor models**: مثالی بمقابلہ حقیقی sensor خصوصیات
- **Actuation**: کامل motors بمقابلہ backlash/compliance

## Gap کو پاٹنا: ثابت شدہ تکنیکیں

### 1. Domain Randomization (DR)

**خیال**: Simulation parameters کو randomize کریں تاکہ حقیقی دنیا صرف ایک اور variation ہو۔

**Randomize کرنے کے لیے parameters:**

```python
import numpy as np

class DomainRandomizer:
    """جامع domain randomization۔"""

    def __init__(self):
        self.params = {}

    def randomize_physics(self):
        """Physics خصوصیات کو randomize کریں۔"""
        # Friction coefficients
        self.params['friction'] = np.random.uniform(0.3, 1.5)

        # Mass (nominal کا ±20%)
        self.params['mass_scale'] = np.random.uniform(0.8, 1.2)

        # Joint damping
        self.params['joint_damping'] = np.random.uniform(0.01, 0.5)

        # Motor طاقت (±10%)
        self.params['motor_scale'] = np.random.uniform(0.9, 1.1)

        # Action delay (0-50ms)
        self.params['action_delay'] = np.random.uniform(0, 0.05)

    def randomize_vision(self):
        """بصری ظہور کو randomize کریں۔"""
        # Lighting
        self.params['light_intensity'] = np.random.uniform(1000, 50000)
        self.params['light_color'] = np.random.random(3)  # RGB

        # Camera
        self.params['exposure'] = np.random.uniform(0.5, 2.0)
        self.params['gamma'] = np.random.uniform(0.8, 1.2)
        self.params['noise_std'] = np.random.uniform(0, 0.02)  # Gaussian noise

        # Object کی شکل
        self.params['object_color'] = np.random.random(3)
        self.params['object_texture'] = np.random.choice([
            "smooth", "rough", "metallic", "matte"
        ])

    def randomize_geometry(self):
        """سائز اور positions کو randomize کریں۔"""
        # Object size (±5%)
        self.params['size_scale'] = np.random.uniform(0.95, 1.05)

        # Spawn position noise (±2cm)
        self.params['position_noise'] = np.random.uniform(-0.02, 0.02, size=3)

        # Orientation noise (±5 درجے)
        self.params['rotation_noise'] = np.random.uniform(-0.087, 0.087, size=3)

    def randomize_all(self):
        """تمام randomizations لگائیں۔"""
        self.randomize_physics()
        self.randomize_vision()
        self.randomize_geometry()
        return self.params
```

**DR کے ساتھ Training:**

```python
def train_with_dr(env, num_episodes=10000):
    """Domain randomization کے ساتھ policy ٹریننگ کریں۔"""
    randomizer = DomainRandomizer()

    for episode in range(num_episodes):
        # اس episode کے لیے domain کو randomize کریں
        params = randomizer.randomize_all()
        env.apply_randomization(params)

        # Episode چلائیں
        obs = env.reset()
        done = False

        while not done:
            action = policy(obs)
            obs, reward, done, info = env.step(action)

        if episode % 100 == 0:
            print(f"Episode {episode}: Avg reward = {avg_reward}")
```

### 2. System Identification

**ہدف**: حقیقی robot خصوصیات کی پیمائش کریں اور simulation کو match کریں۔

**مرحلہ 1: Friction کی شناخت**

```python
def identify_friction(robot):
    """
    مستقل force لگائیں، terminal velocity ناپیں۔
    friction_coef = force / (mass * g)
    """
    forces = [0.5, 1.0, 1.5, 2.0]  # Newtons
    velocities = []

    for force in forces:
        robot.apply_force(force)
        time.sleep(2.0)  # Terminal velocity تک پہنچنے کا انتظار کریں
        vel = robot.get_velocity()
        velocities.append(vel)

    # Linear regression: F = μ * m * g + friction_loss
    # آسان کیا ہوا: μ ≈ F / (m * g)
    mass = 5.0  # kg (معلوم)
    g = 9.81
    friction_coef = np.mean(forces) / (mass * g)

    return friction_coef
```

**مرحلہ 2: Simulation کو اپ ڈیٹ کریں**

```xml
<!-- شناخت شدہ parameters کے ساتھ URDF/USD کو اپ ڈیٹ کریں -->
<gazebo reference="link">
  <mu1>0.68</mu1>  <!-- شناخت شدہ friction -->
  <mu2>0.68</mu2>
</gazebo>
```

### 3. Privileged Learning + Adaptation

**خیال**: کامل sim معلومات کے ساتھ ٹریننگ کریں، پھر deployment پر adapt کریں۔

```python
class PrivilegedPolicy:
    """Policy جو training کے دوران privileged معلومات استعمال کرتی ہے۔"""

    def __init__(self):
        # Student policy (deployed)
        self.student = StudentPolicy(obs_dim=10, action_dim=4)

        # Teacher policy (صرف training، privileged info رکھتی ہے)
        self.teacher = TeacherPolicy(obs_dim=10, priv_dim=20, action_dim=4)

    def train_step(self, obs, privileged_info, true_action):
        """دونوں policies ٹریننگ کریں۔"""
        # Teacher privileged info استعمال کرتا ہے (حقیقی object mass، friction، وغیرہ)
        teacher_action = self.teacher(obs, privileged_info)

        # Student privileged info کے بغیر teacher سے match کرنے کی کوشش کرتا ہے
        student_action = self.student(obs)

        # Losses
        teacher_loss = mse_loss(teacher_action, true_action)
        distillation_loss = mse_loss(student_action, teacher_action)

        total_loss = teacher_loss + distillation_loss
        return total_loss

    def deploy(self, obs):
        """Deployment پر، صرف student استعمال کریں۔"""
        return self.student(obs)
```

**Privileged معلومات کی مثالیں:**
- حقیقی object mass، friction coefficients
- Ground-truth object poses (بمقابلہ noisy perception)
- مستقبل کا trajectory (پیشن گوئی کے کاموں کے لیے)
- پوشیدہ state (joint forces، contact points)

### 4. حقیقی Data پر Fine-Tuning

**حکمت عملی**: Sim میں ٹریننگ کریں، چھوٹے حقیقی dataset کے ساتھ fine-tune کریں۔

```python
# مرحلہ 1: Simulation میں Pre-train کریں (لاکھوں samples)
policy = train_in_simulation(num_steps=10_000_000)

# مرحلہ 2: حقیقی data جمع کریں (سیکڑوں samples)
real_data = collect_real_robot_data(num_episodes=100)

# مرحلہ 3: حقیقی data پر Fine-tune کریں
policy = fine_tune(policy, real_data, num_epochs=50, lr=1e-5)
```

**بہترین طریقے:**
- Catastrophic forgetting سے بچنے کے لیے کم learning rate استعمال کریں
- Simulation سے 90% training data رکھیں
- حقیقی data کو failure modes پر focus کریں

### 5. Residual Learning

**خیال**: Sim policy کے اوپر correction سیکھیں۔

```python
class ResidualPolicy:
    """Sim policy + سیکھا ہوا residual۔"""

    def __init__(self, sim_policy):
        self.sim_policy = sim_policy  # Frozen
        self.residual_network = ResidualNet(obs_dim=10, action_dim=4)

    def forward(self, obs):
        # Sim policy action حاصل کریں
        sim_action = self.sim_policy(obs)

        # Residual (correction) حساب کریں
        residual = self.residual_network(obs)

        # حتمی action = sim + residual
        action = sim_action + residual
        return action
```

**Training:**
- حقیقی robot پر residual network ٹریننگ کریں
- Sim policy کا علم رکھتا ہے، صرف corrections سیکھتا ہے

## مقداری Transfer Evaluation

### میٹرکس

**1. Success Rate**
```
Success Rate = (کامیاب آزمائشیں / کل آزمائشیں) × 100%
```

**2. Sim-to-Real Performance Ratio**
```
Transfer Ratio = حقیقی کارکردگی / Sim کارکردگی
```
- Ratio = 1.0: کامل transfer
- Ratio < 0.7: خراب transfer (بہتری کی ضرورت)
- Ratio > 0.9: بہترین transfer

**3. Sample Efficiency**
```
Samples Needed = 90% sim کارکردگی تک پہنچنے کے لیے حقیقی samples
```

**4. Task-Specific میٹرکس**
- **Grasping**: Grasp success rate، grasp stability
- **Navigation**: ہدف تک پہنچنے کی کامیابی، collision rate
- **Manipulation**: Task مکمل کرنے کا وقت، precision

### Evaluation پروٹوکول

```python
def evaluate_transfer(policy, real_env, num_trials=100):
    """Sim-to-real transfer کا جائزہ لیں۔"""
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

        # نتیجہ ریکارڈ کریں
        if info["success"]:
            successes += 1
            completion_times.append(steps)
        else:
            failure_type = info["failure_reason"]
            failures[failure_type] += 1

    # میٹرکس حساب کریں
    success_rate = successes / num_trials
    avg_time = np.mean(completion_times) if completion_times else None

    return {
        "success_rate": success_rate,
        "avg_completion_time": avg_time,
        "failure_breakdown": failures
    }
```

## کیس اسٹڈی: Grasping Transfer

### Simulation Setup

```python
# Isaac Sim میں grasping policy ٹریننگ کریں
env = GraspingEnv(
    num_envs=2048,
    domain_randomization=True,
    object_types=["cube", "cylinder", "sphere", "irregular"],
    object_textures=textures_library,  # 100+ textures
    lighting_range=(5000, 50000),
    friction_range=(0.3, 1.5)
)

# PPO کے ساتھ ٹریننگ کریں
model = PPO("MultiInputPolicy", env, learning_rate=3e-4)
model.learn(total_timesteps=5_000_000)
```

### حقیقی Robot Deployment

```python
# Sim-trained policy لوڈ کریں
policy = load_policy("grasp_policy_sim.pth")

# حقیقی robot environment
real_env = RealRobotEnv(
    camera_topic="/camera/rgb",
    robot_ip="192.168.1.10"
)

# Evaluate کریں
results = evaluate_transfer(policy, real_env, num_trials=50)
print(f"Sim-to-Real Success Rate: {results['success_rate']:.1%}")
```

**عام نتائج:**
- کوئی DR نہیں: 30-40% کامیابی
- DR کے ساتھ: 70-85% کامیابی
- DR + Fine-tuning: 85-95% کامیابی

## باب 3 Assessment پروجیکٹ

**کام**: manipulation task کے لیے مکمل Isaac Sim pipeline بنائیں

### پروجیکٹ کی ضروریات (100 پوائنٹس)

#### حصہ 1: Scene Setup (15 پوائنٹس)
- Warehouse/factory environment بنائیں
- رکاوٹیں، مختلف lighting شامل کریں
- Manipulation کے لیے 3+ object types شامل کریں
- Custom USD models (بونس: CAD سے import کریں)

#### حصہ 2: Synthetic Data Generation (25 پوائنٹس)
- Annotations کے ساتھ 5000+ تصاویر تیار کریں
- جامع domain randomization نافذ کریں:
  - Lighting (3+ ذرائع، مختلف intensity/رنگ)
  - Object poses، سائز، textures
  - Camera parameters، noise
- COCO یا custom format میں export کریں
- تقسیم: 80% train، 10% val، 10% test

#### حصہ 3: Vision Model Training (25 پوائنٹس)
- Object detection یا segmentation model ٹریننگ کریں
- صرف synthetic data استعمال کریں
- Validation set پر میٹرکس رپورٹ کریں:
  - Detection کے لیے mAP@0.5
  - Segmentation کے لیے IoU
- Test images پر predictions کو visualize کریں

#### حصہ 4: RL Policy Training (25 پوائنٹس)
- Manipulation task کی تعریف کریں (pick-and-place، reaching، وغیرہ)
- Domain randomization کے ساتھ environment نافذ کریں
- RL algorithm کے ساتھ policy ٹریننگ کریں (PPO/SAC/وغیرہ)
- Sim میں کامیاب task execution کا مظاہرہ کریں
- Training curves، success rate رپورٹ کریں

#### حصہ 5: دستاویزات (10 پوائنٹس)
- Setup instructions کے ساتھ README
- Architecture diagram (scene، sensors، algorithms)
- Training/evaluation رپورٹیں
- Demo video (زیادہ سے زیادہ 5 منٹ مکمل pipeline دکھاتے ہوئے)

### ڈیلیور ایبلز

1. **GitHub Repository**:
   - تمام source code
   - Dataset generation scripts
   - Training scripts
   - Trained models/policies

2. **Dataset**:
   - تیار شدہ dataset کا لنک (cloud storage ٹھیک ہے)
   - Repository میں 100 sample images

3. **رپورٹ** (PDF، 3-5 صفحات):
   - طریقہ کار
   - Domain randomization حکمت عملی
   - Training میٹرکس اور curves
   - چیلنجز اور حل

4. **Demo Video**:
   - Scene walkthrough
   - Dataset generation process
   - Model/policy عمل میں
   - مقداری نتائج

### Grading Rubric

| جزو | پوائنٹس | معیار |
|-----------|--------|----------|
| **Scene Quality** | 15 | حقیقت پسندانہ، متنوع، اچھی طرح روشن |
| **Dataset Quality** | 15 | بڑا، متنوع، صحیح طریقے سے annotated |
| **DR Implementation** | 10 | جامع randomization |
| **Model Performance** | 15 | درستگی کی حدود کو پورا کرتا ہے |
| **RL Policy** | 15 | Sim میں Task کامیابی > 80% |
| **Code Quality** | 10 | صاف، دستاویز شدہ، قابل تکرار |
| **دستاویزات** | 10 | واضح، مکمل، اچھی طرح لکھا ہوا |
| **تخلیقیت** | 10 | نیا طریقہ یا اضافی خصوصیات |

**بونس مواقع** (+زیادہ سے زیادہ 20 پوائنٹس):
- حقیقی robot پر deploy کریں (+15)
- Multi-task learning (+10)
- Custom physics simulation (+10)
- SLAM integration (+10)

### مثال کے پروجیکٹس

**مثال 1: Bin Picking**
- Task: بکھرے ہوئے bin سے parts اٹھائیں
- Objects: پیچ، نٹ، واشر (مختلف سائز)
- Vision: Detection کے لیے YOLOv8
- Policy: Pick-and-place کے لیے PPO

**مثال 2: Quality Inspection**
- Task: تیار شدہ parts پر خرابیوں کا پتہ لگائیں
- Dataset: خرابی annotations کے ساتھ 10K تصاویر
- Model: Semantic segmentation (U-Net)
- Deployment: Real-time inference کے لیے ROS 2 node

**مثال 3: Warehouse Navigation**
- Task: Shelf تک navigate کریں، چیز بازیافت کریں
- Environment: Aisles کے ساتھ 50x50m warehouse
- Vision: رکاوٹوں سے بچنے کے لیے depth camera
- Policy: Navigation + grasping کے لیے SAC

## ہفتہ 10 عملی مشق

**کام**: DR حکمت عملیوں کو نافذ اور موازنہ کریں

1. **بغیر** DR کے baseline policy ٹریننگ کریں
2. صرف lighting DR کے ساتھ policy ٹریننگ کریں
3. مکمل DR کے ساتھ policy ٹریننگ کریں (lighting + physics + vision)
4. متغیر test environments میں تینوں کا جائزہ لیں
5. کارکردگی کا موازنہ رپورٹ کریں

**متوقع نتیجہ**: مکمل DR policy environment کی تبدیلیوں کے لیے سب سے زیادہ مضبوط ہونی چاہیے۔

## وسائل

- [Sim-to-Real Transfer Survey](https://arxiv.org/abs/2009.13303)
- [Domain Randomization for Transferring Deep Neural Networks](https://arxiv.org/abs/1703.06907)
- [Learning Dexterous In-Hand Manipulation](https://arxiv.org/abs/1808.00177) - OpenAI کا Rubik's Cube
- [NVIDIA Isaac Sim Examples](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)
- [System Identification Techniques](https://stanford.edu/class/ee363/sysid.pdf)

## باب 3 کا خلاصہ

باب 3 مکمل کرنے پر مبارکباد! آپ نے سیکھا ہے:

✅ NVIDIA Isaac Sim سیٹ اپ اور navigation
✅ USD scene creation اور robot import
✅ Sensor data کے لیے ROS 2 integration
✅ Replicator کے ساتھ synthetic data generation
✅ Domain randomization حکمت عملی
✅ Isaac Gym کے ساتھ بہت زیادہ parallel RL
✅ Sim-to-real transfer تکنیکیں
✅ Sim سے deployment تک مکمل ML pipeline

## اگلے قدم

**آگے کیا ہے:**
- باب 3 کا assessment پروجیکٹ مکمل کریں
- ضرورت کے مطابق Isaac Sim تصورات کا جائزہ لیں
- باب 4 کے لیے تیاری کریں: [Vision-Language-Action Models & Capstone](../04-vla/index.md)

باب 4 ہر چیز کو اکٹھا کرے گا: multimodal AI، end-to-end robot systems، اور آپ کا حتمی capstone پروجیکٹ!

---

## 📝 ہفتہ وار Quiz

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! Quiz multiple choice ہے، خودکار طور پر score ہوتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 10 Quiz لیں →](/quiz?week=10)**
