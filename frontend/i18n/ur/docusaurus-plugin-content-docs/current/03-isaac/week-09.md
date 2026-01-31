# ہفتہ 9: Synthetic Data Generation & Isaac Gym

## جائزہ

یہ ہفتہ Isaac Sim کی دو طاقتور صلاحیتوں کو دریافت کرتا ہے: Vision models ٹریننگ کرنے کے لیے **Synthetic Data Generation** (SDG) اور بہت زیادہ parallel reinforcement learning کے لیے **Isaac Gym**۔ آپ سیکھیں گے کہ حقیقت پسندانہ training datasets کیسے بنائیں اور robot policies کو مکمل طور پر simulation میں کیسے train کریں۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- AI/ML کے لیے synthetic data کی قدر سمجھیں
- Dataset generation کے لیے Isaac Sim Replicator استعمال کریں
- Domain-randomized training data بنائیں
- Annotated datasets تیار کریں (bounding boxes، segmentation masks)
- Reinforcement learning کے لیے Isaac Gym سیٹ اپ کریں
- سادہ RL policy ٹریننگ کریں (reaching، grasping)
- Sim-to-real transfer کی تیاری کا جائزہ لیں

## Synthetic Data کیوں؟

### Robotics میں Data کا مسئلہ

**روایتی طریقہ:**
1. جسمانی robot بنائیں ($10K-$1M)
2. دستی طور پر data جمع کریں (ہفتے/مہینے)
3. دستی طور پر data کو label کریں (مہنگا، غلطی کا شکار)
4. Model ٹریننگ کریں
5. جب model نئے منظرناموں میں ناکام ہو تو دہرائیں

**Synthetic data طریقہ:**
1. Simulation بنائیں (دن)
2. خودکار طور پر لاکھوں samples تیار کریں (گھنٹے)
3. خودکار طور پر کامل labels (مفت)
4. Model ٹریننگ کریں
5. نئے منظرناموں کے لیے domain randomization شامل کریں

### Synthetic Data کے فوائد

| پہلو | حقیقی Data | Synthetic Data |
|--------|-----------|----------------|
| **لاگت** | $$$$ (hardware، محنت) | $ (صرف compute) |
| **رفتار** | سست (جسمانی جمع) | تیز (parallel generation) |
| **پیمانہ** | ہزاروں تصاویر | لاکھوں تصاویر |
| **Labels** | دستی ($0.10-$1/image) | خودکار (مفت، کامل) |
| **تنوع** | جسمانی سیٹ اپ سے محدود | لامحدود (domain randomization) |
| **حفاظت** | نقصان کا خطرہ | خطرے سے پاک |
| **Edge cases** | پکڑنا مشکل | بنانا آسان |

### چیلنجز اور حل

**چیلنج 1: Sim-to-Real Gap**
- Simulated تصاویر حقیقت کے مقابلے میں "جعلی" نظر آتی ہیں
- **حل**: Domain randomization (lighting، textures، camera params)

**چیلنج 2: Physics Mismatch**
- Simulated physics حقیقی دنیا سے مختلف ہے
- **حل**: System identification، حقیقی data پر fine-tuning

**چیلنج 3: Simulation پر Overfitting**
- Model sim میں کام کرتا ہے لیکن حقیقی robot پر ناکام ہوتا ہے
- **حل**: متنوع randomization، sim-to-real transfer تکنیکیں

## Isaac Sim Replicator

**Replicator** Isaac Sim کا synthetic data generation framework ہے۔

### اہم صلاحیتیں

- **Randomization**: Materials، lighting، camera params، object poses
- **Annotations**: Bounding boxes، segmentation، depth، normals
- **Scalability**: Parallel میں ہزاروں تصاویر تیار کریں
- **Formats**: COCO، KITTI، Custom JSON

### Replicator Workflow

```
1. بنیادی scene بنائیں
   ↓
2. Randomizers کی تعریف کریں (lighting، materials، poses)
   ↓
3. Writers کی تعریف کریں (annotations محفوظ کریں)
   ↓
4. Generation loop چلائیں
   ↓
5. Dataset export کریں
```

## Synthetic Dataset بنانا

### مثال: Object Detection Dataset

**ہدف**: میز پر boxes کا پتہ لگانے کے لیے YOLOv8 ٹریننگ کریں

#### مرحلہ 1: بنیادی Scene بنائیں

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})  # رفتار کے لیے کوئی GUI نہیں

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims import GeometryPrim
from omni.replicator.core import randomizer, Writer
import omni.replicator.core as rep
import numpy as np

# دنیا بنائیں
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# میز بنائیں
table = world.scene.add(
    VisualCuboid(
        prim_path="/World/Table",
        name="table",
        position=np.array([0, 0, 0.5]),
        size=np.array([1.0, 1.0, 0.05]),
        color=np.array([0.5, 0.3, 0.1])  # بھورا
    )
)

# میز کو دیکھنے والا camera بنائیں
camera = rep.create.camera(
    position=(0, -2, 1.5),
    look_at=(0, 0, 0.5)
)

# روشنی بنائیں
light = rep.create.light(
    light_type="Sphere",
    intensity=30000,
    position=(2, 2, 3),
    scale=0.5
)
```

#### مرحلہ 2: Randomizers کی تعریف کریں

```python
import omni.replicator.core as rep

# آبجیکٹس کی positions کو randomize کریں
def randomize_objects():
    """میز پر 1-5 boxes بے ترتیب طور پر رکھیں۔"""
    num_objects = np.random.randint(1, 6)

    for i in range(num_objects):
        # میز پر بے ترتیب position
        x = np.random.uniform(-0.4, 0.4)
        y = np.random.uniform(-0.4, 0.4)
        z = 0.525  # میز سے بالکل اوپر

        # بے ترتیب سائز
        size = np.random.uniform(0.05, 0.15)

        # بے ترتیب رنگ
        color = np.random.random(3)

        # Box بنائیں
        box = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Box_{i}",
                name=f"box_{i}",
                position=np.array([x, y, z]),
                size=np.array([size, size, size]),
                color=color
            )
        )

# Lighting کو randomize کریں
def randomize_lighting():
    """روشنی کی شدت اور position میں تبدیلی کریں۔"""
    intensity = np.random.uniform(20000, 40000)
    x = np.random.uniform(-3, 3)
    y = np.random.uniform(-3, 3)
    z = np.random.uniform(2, 4)

    # روشنی کو اپ ڈیٹ کریں (replicator API استعمال کریں)
    with rep.new_layer():
        light = rep.get.prims(path_pattern="/World/Lights/*")
        with light:
            rep.modify.pose(position=(x, y, z))
            rep.modify.attribute("intensity", intensity)

# Camera کو randomize کریں
def randomize_camera():
    """میز کے گرد camera position میں تبدیلی کریں۔"""
    # کروی coordinates
    radius = np.random.uniform(1.5, 2.5)
    theta = np.random.uniform(-np.pi/4, np.pi/4)  # ±45 درجے
    phi = np.random.uniform(np.pi/6, np.pi/3)     # 30-60 درجے بلندی

    x = radius * np.cos(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.cos(phi)
    z = radius * np.sin(phi)

    with rep.new_layer():
        camera = rep.get.prims(path_pattern="/World/Camera")
        with camera:
            rep.modify.pose(position=(x, y, z), look_at=(0, 0, 0.5))
```

#### مرحلہ 3: Annotators رجسٹر کریں

```python
# Annotations فعال کریں
rp = rep.create.render_product(camera, (640, 480))

# RGB تصاویر
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

#### مرحلہ 4: Custom Writer (COCO Format)

```python
import omni.replicator.core as rep
import json
import os
from PIL import Image

class COCOWriter(rep.Writer):
    """COCO format میں dataset export کریں۔"""

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
        """ہر frame کے لیے کال کیا جاتا ہے۔"""
        # RGB image محفوظ کریں
        rgb = data["rgb"]
        img_filename = f"image_{self.image_id:06d}.png"
        img_path = f"{self.output_dir}/images/{img_filename}"
        Image.fromarray(rgb).save(img_path)

        # Image metadata شامل کریں
        self.coco_data["images"].append({
            "id": self.image_id,
            "file_name": img_filename,
            "width": rgb.shape[1],
            "height": rgb.shape[0]
        })

        # Bounding box annotations شامل کریں
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
        """آخری frame کے بعد کال کیا جاتا ہے۔"""
        # COCO JSON محفوظ کریں
        with open(f"{self.output_dir}/annotations.json", "w") as f:
            json.dump(self.coco_data, f, indent=2)

        print(f"Dataset saved to {self.output_dir}")
        print(f"Total images: {self.image_id}")
        print(f"Total annotations: {self.annot_id}")

# Writer رجسٹر کریں
writer = COCOWriter(output_dir="./dataset_boxes")
writer.attach(rp)
```

#### مرحلہ 5: Generation چلائیں

```python
# Generation loop
num_frames = 1000

world.reset()

for i in range(num_frames):
    # Scene کو randomize کریں
    randomize_objects()
    randomize_lighting()
    randomize_camera()

    # Physics step (اشیاء کو settle ہونے دیں)
    for _ in range(10):
        world.step(render=False)

    # Frame capture کریں
    world.step(render=True)

    # Replicator write کو trigger کریں
    rep.orchestrator.step()

    if i % 100 == 0:
        print(f"Generated {i}/{num_frames} frames")

# Finalize
writer.on_final_frame()
simulation_app.close()
```

**نتیجہ**: YOLO ٹریننگ کے لیے تیار COCO annotations کے ساتھ 1000 تصاویر!

## Domain Randomization کے بہترین طریقے

### 1. Lighting Randomization

```python
# بے ترتیب رنگوں کے ساتھ متعدد روشنیاں
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
# آبجیکٹس پر بے ترتیب materials لگائیں
materials = [
    "omni://localhost/NVIDIA/Materials/vMaterials_2/Ground/textures/aggregate_exposed_diff.jpg",
    "omni://localhost/NVIDIA/Materials/vMaterials_2/Wood/textures/wood_cherry_diff.jpg",
    # مزید material paths شامل کریں
]

def randomize_materials():
    boxes = rep.get.prims(semantics=[("class", "box")])
    with boxes:
        rep.randomizer.materials(materials)
```

### 3. Camera Randomization

```python
# حقیقی camera noise کی نقل کریں
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
# بے ترتیب HDRI backgrounds استعمال کریں
hdris = [
    "omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCG_com_WarehouseInterior2.hdr",
    "omniverse://localhost/NVIDIA/Assets/Skies/Outdoor/kloppenheim_06_4k.hdr",
]

with rep.new_layer():
    dome_light = rep.create.light(light_type="Dome")
    with dome_light:
        rep.randomizer.texture(hdris)
```

## Isaac Gym: بہت زیادہ Parallel RL

**Isaac Gym** ایک GPU پر ہزاروں robot policies کی بیک وقت ٹریننگ کو ممکن بناتا ہے۔

### اہم تصورات

- **Vectorized environments**: بیک وقت 1000+ instances چلائیں
- **GPU physics**: GPU پر تمام simulation (کوئی CPU bottleneck نہیں)
- **GPU tensors**: Observations/actions GPU پر رہتے ہیں (کوئی CPU↔GPU transfer نہیں)
- **Fast**: دنوں کے بجائے منٹوں میں policies ٹریننگ کریں

### Isaac Gym بمقابلہ روایتی RL

| میٹرک | روایتی (CPU) | Isaac Gym (GPU) |
|--------|-------------------|-----------------|
| **Parallel envs** | 8-16 | 1024-8192 |
| **Timesteps/sec** | 1000-5000 | 100K-1M |
| **Training time (Reach)** | 24 گھنٹے | 5 منٹ |
| **Hardware** | Multi-core CPU | Single RTX GPU |

### سادہ Reach Task

**ہدف**: بے ترتیب ہدف positions تک پہنچنے کے لیے robot arm ٹریننگ کریں

#### Environment Setup

```python
from omni.isaac.gym import VecEnvBase
import torch
import numpy as np

class ReachEnv(VecEnvBase):
    """سادہ reaching task۔"""

    def __init__(self, num_envs=1024, device="cuda:0"):
        self.num_envs = num_envs
        self.device = device

        # Observation: [joint positions (7), target position (3)]
        self.num_obs = 10

        # Action: target joint positions (7)
        self.num_actions = 7

        super().__init__(num_envs=num_envs)

        # Targets شروع کریں
        self.targets = torch.zeros((num_envs, 3), device=device)

    def reset(self):
        """تمام environments کو reset کریں۔"""
        # ہدف positions کو randomize کریں
        self.targets = torch.rand((self.num_envs, 3), device=self.device)
        self.targets[:, 0] = self.targets[:, 0] * 0.6 + 0.2  # X: 0.2-0.8
        self.targets[:, 1] = (self.targets[:, 1] - 0.5) * 0.6 # Y: -0.3-0.3
        self.targets[:, 2] = self.targets[:, 2] * 0.5 + 0.2   # Z: 0.2-0.7

        # Robot کو home position پر reset کریں
        home_joints = torch.tensor([0, -1.0, 0, -2.2, 0, 2.4, 0.8], device=self.device)
        self.joint_positions = home_joints.repeat(self.num_envs, 1)

        return self.get_observations()

    def get_observations(self):
        """موجودہ observations حاصل کریں۔"""
        # Joint positions اور target کو concatenate کریں
        obs = torch.cat([self.joint_positions, self.targets], dim=1)
        return obs

    def step(self, actions):
        """Actions apply کریں اور simulation step کریں۔"""
        # Actions target joint positions ہیں
        self.joint_positions = actions

        # End-effector position حاصل کریں (آسان کیا ہوا - حقیقت میں forward kinematics استعمال کریں)
        ee_pos = self.compute_ee_position(self.joint_positions)

        # Reward حساب کریں: ہدف سے منفی فاصلہ
        distance = torch.norm(ee_pos - self.targets, dim=1)
        rewards = -distance

        # Episode مکمل اگر ہدف تک پہنچ گیا (فاصلہ < 0.05m)
        dones = distance < 0.05

        # نئے observations حاصل کریں
        obs = self.get_observations()

        return obs, rewards, dones, {}

    def compute_ee_position(self, joint_positions):
        """آسان کیا ہوا forward kinematics۔"""
        # حقیقت میں، مناسب FK استعمال کریں
        # یہاں، صرف مظاہرے کے لیے تخمینہ
        return torch.rand((self.num_envs, 3), device=self.device)
```

#### PPO کے ساتھ Training

```python
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize

# Environment بنائیں
env = ReachEnv(num_envs=2048)

# Observations اور rewards کو normalize کریں
env = VecNormalize(env, norm_obs=True, norm_reward=True)

# PPO agent بنائیں
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=16,  # تیز updates کے لیے چھوٹا
    batch_size=4096,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.0,
    verbose=1,
    device="cuda"
)

# Train کریں
model.learn(total_timesteps=1_000_000)

# Model محفوظ کریں
model.save("reach_policy")
```

**Training RTX 3080 پر ~5 منٹ میں مکمل ہوتی ہے!**

### حقیقی Isaac Gym مثال (Cartpole)

Isaac Gym میں پہلے سے بنائے ہوئے tasks شامل ہیں:

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/standalone_examples/api/omni.isaac.gym

# Cartpole مثال چلائیں
python cartpole.py
```

Parallel میں 2048 cartpoles ٹریننگ کرتا ہے!

## ہفتہ 9 عملی پروجیکٹ

**کام**: Synthetic dataset بنائیں اور سادہ model ٹریننگ کریں

**حصہ 1: Synthetic Dataset (50 پوائنٹس)**
- Scene: 3-6 رنگین cubes کے ساتھ میز
- Randomization: Lighting (3 ذرائع)، cube positions، cube رنگ
- 2000 تصاویر تیار کریں (640x480)
- Annotations: COCO format میں bounding boxes
- Dataset کو disk پر محفوظ کریں

**حصہ 2: Model Training (50 پوائنٹس)**
- Synthetic data پر YOLOv8 یا Faster R-CNN ٹریننگ کریں
- 200-image validation set پر evaluate کریں
- mAP (mean Average Precision) رپورٹ کریں
- Sim-to-real gap کا جائزہ لینے کے لیے حقیقی تصاویر پر جانچ کریں (اگر دستیاب ہوں)

**ڈیلیور ایبلز:**
- Dataset generation کے لیے Replicator script
- Training script اور logs
- Trained model weights
- میٹرکس کے ساتھ evaluation رپورٹ

**بونس (+20 پوائنٹس):**
- Custom domain randomization نافذ کریں (distractor objects، camera noise)
- سادہ manipulation task کے لیے RL policy ٹریننگ کریں

## وسائل

- [Replicator Documentation](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)
- [Isaac Gym Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/index.html)
- [Synthetic Data Generation Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)
- [Domain Randomization Paper](https://arxiv.org/abs/1703.06907)
- [Isaac Gym Benchmark](https://leggedrobotics.github.io/rl-games/)

## اگلے قدم

بہترین کام! اب آپ synthetic data generation اور parallel RL training سمجھتے ہیں۔

اگلا ہفتہ: [ہفتہ 10: Sim-to-Real Transfer & Chapter 3 Project](week-10.md)

ہم sim-to-real gap سے نمٹیں گے اور ایک جامع Isaac Sim پروجیکٹ مکمل کریں گے!

---

## 📝 ہفتہ وار Quiz

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! Quiz multiple choice ہے، خودکار طور پر score ہوتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 9 Quiz لیں →](/quiz?week=9)**
