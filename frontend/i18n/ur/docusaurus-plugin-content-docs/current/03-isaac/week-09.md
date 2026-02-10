# ہفتہ 9: سنتھیٹک ڈیٹا جنریشن اور آئزک جم

## جائزہ

یہ ہفتہ آئزک سم کی دو طاقتور صلاحیتوں کو دریافت کرتا ہے: ویژن ماڈلز ٹریننگ کرنے کے لیے **سنتھیٹک ڈیٹا جنریشن** (ایس ڈی جی) اور بہت زیادہ پیرالل ری انفورسمنٹ لرننگ کے لیے **آئزک جم**۔ آپ سیکھیں گے کہ حقیقت پسندانہ ٹریننگ ڈیٹاسیٹس کیسے بنائیں اور روبوٹ پالیسیز کو مکمل طور پر سمیولیشن میں کیسے ٹرین کریں۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- اے آئی/ایم ایل کے لیے سنتھیٹک ڈیٹا کی قدر سمجھیں
- ڈیٹاسیٹ جنریشن کے لیے آئزک سم ریپلیکیٹر استعمال کریں
- ڈومین-رینڈمائزڈ ٹریننگ ڈیٹا بنائیں
- اینوٹیٹڈ ڈیٹاسیٹس تیار کریں (باؤنڈنگ باکسز، سیگمینٹیشن ماسکس)
- ری انفورسمنٹ لرننگ کے لیے آئزک جم سیٹ اپ کریں
- سادہ آر ایل پالیسی ٹریننگ کریں (ریچنگ، گراسپنگ)
- سم ٹو ریئل ٹرانسفر کی تیاری کا جائزہ لیں

## سنتھیٹک ڈیٹا کیوں؟

### روبوٹکس میں ڈیٹا کا مسئلہ

**روایتی طریقہ:**
1. جسمانی روبوٹ بنائیں ($10 ہزار-$10 لاکھ)
2. دستی طور پر ڈیٹا جمع کریں (ہفتے/مہینے)
3. دستی طور پر ڈیٹا کو لیبل کریں (مہنگا، غلطی کا شکار)
4. ماڈل ٹریننگ کریں
5. جب ماڈل نئے منظرناموں میں ناکام ہو تو دہرائیں

**سنتھیٹک ڈیٹا طریقہ:**
1. سمیولیشن بنائیں (دن)
2. خودکار طور پر لاکھوں سیمپلز تیار کریں (گھنٹے)
3. خودکار طور پر کامل لیبلز (مفت)
4. ماڈل ٹریننگ کریں
5. نئے منظرناموں کے لیے ڈومین رینڈمائزیشن شامل کریں

### سنتھیٹک ڈیٹا کے فوائد

| پہلو | حقیقی ڈیٹا | سنتھیٹک ڈیٹا |
|--------|-----------|----------------|
| **لاگت** | $$$$ (ہارڈویئر، محنت) | $ (صرف کمپیوٹ) |
| **رفتار** | سست (جسمانی جمع) | تیز (پیرالل جنریشن) |
| **پیمانہ** | ہزاروں تصاویر | لاکھوں تصاویر |
| **لیبلز** | دستی ($0.10-$1/تصویر) | خودکار (مفت، کامل) |
| **تنوع** | جسمانی سیٹ اپ سے محدود | لامحدود (ڈومین رینڈمائزیشن) |
| **حفاظت** | نقصان کا خطرہ | خطرے سے پاک |
| **ایج کیسز** | پکڑنا مشکل | بنانا آسان |

### چیلنجز اور حل

**چیلنج 1: سم ٹو ریئل گیپ**
- سمیولیٹڈ تصاویر حقیقت کے مقابلے میں "جعلی" نظر آتی ہیں
- **حل**: ڈومین رینڈمائزیشن (لائٹنگ، ٹیکسچرز، کیمرا پیرامز)

**چیلنج 2: فزکس مسمیچ**
- سمیولیٹڈ فزکس حقیقی دنیا سے مختلف ہے
- **حل**: سسٹم آئیڈینٹیفکیشن، حقیقی ڈیٹا پر فائن ٹیوننگ

**چیلنج 3: سمیولیشن پر اوور فٹنگ**
- ماڈل سم میں کام کرتا ہے لیکن حقیقی روبوٹ پر ناکام ہوتا ہے
- **حل**: متنوع رینڈمائزیشن، سم ٹو ریئل ٹرانسفر تکنیکیں

## آئزک سم ریپلیکیٹر

**ریپلیکیٹر** آئزک سم کا سنتھیٹک ڈیٹا جنریشن فریم ورک ہے۔

### اہم صلاحیتیں

- **رینڈمائزیشن**: میٹیریلز، لائٹنگ، کیمرا پیرامز، آبجیکٹ پوزز
- **اینوٹیشنز**: باؤنڈنگ باکسز، سیگمینٹیشن، ڈیپتھ، نارملز
- **اسکیلیبیلیٹی**: پیرالل میں ہزاروں تصاویر تیار کریں
- **فارمیٹس**: کوکو، کٹی، کسٹم جے سن

### ریپلیکیٹر ورک فلو

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

## سنتھیٹک ڈیٹاسیٹ بنانا

### مثال: آبجیکٹ ڈیٹیکشن ڈیٹاسیٹ

**ہدف**: میز پر باکسز کا پتہ لگانے کے لیے یولو وی 8 ٹریننگ کریں

#### مرحلہ 1: بنیادی سین بنائیں

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

#### مرحلہ 2: رینڈمائزرز کی تعریف کریں

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

#### مرحلہ 3: اینوٹیٹرز رجسٹر کریں

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

#### مرحلہ 4: کسٹم رائٹر (کوکو فارمیٹ)

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

#### مرحلہ 5: جنریشن چلائیں

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

**نتیجہ**: یولو ٹریننگ کے لیے تیار کوکو اینوٹیشنز کے ساتھ 1000 تصاویر!

## ڈومین رینڈمائزیشن کے بہترین طریقے

### 1. لائٹنگ رینڈمائزیشن

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

### 2. ٹیکسچر رینڈمائزیشن

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

### 3. کیمرا رینڈمائزیشن

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

### 4. بیک گراؤنڈ رینڈمائزیشن

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

## آئزک جم: بہت زیادہ پیرالل آر ایل

**آئزک جم** ایک جی پی یو پر ہزاروں روبوٹ پالیسیز کی بیک وقت ٹریننگ کو ممکن بناتا ہے۔

### اہم تصورات

- **ویکٹرائزڈ انوائرنمنٹس**: بیک وقت 1000+ انسٹینسز چلائیں
- **جی پی یو فزکس**: جی پی یو پر تمام سمیولیشن (کوئی سی پی یو باٹل نیک نہیں)
- **جی پی یو ٹینسرز**: آبزرویشنز/ایکشنز جی پی یو پر رہتے ہیں (کوئی سی پی یو↔جی پی یو ٹرانسفر نہیں)
- **تیز**: دنوں کے بجائے منٹوں میں پالیسیز ٹریننگ کریں

### آئزک جم بمقابلہ روایتی آر ایل

| میٹرک | روایتی (سی پی یو) | آئزک جم (جی پی یو) |
|--------|-------------------|-----------------|
| **پیرالل انوز** | 8-16 | 1024-8192 |
| **ٹائم سٹیپس/سیکنڈ** | 1000-5000 | 100 ہزار-10 لاکھ |
| **ٹریننگ ٹائم (ریچ)** | 24 گھنٹے | 5 منٹ |
| **ہارڈویئر** | ملٹی کور سی پی یو | سنگل آر ٹی ایکس جی پی یو |

### سادہ ریچ ٹاسک

**ہدف**: بے ترتیب ہدف پوزیشنز تک پہنچنے کے لیے روبوٹ آرم ٹریننگ کریں

#### انوائرنمنٹ سیٹ اپ

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

#### پی پی او کے ساتھ ٹریننگ

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

**ٹریننگ آر ٹی ایکس 3080 پر ~5 منٹ میں مکمل ہوتی ہے!**

### حقیقی آئزک جم مثال (کارٹ پول)

آئزک جم میں پہلے سے بنائے ہوئے ٹاسکس شامل ہیں:

```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1/standalone_examples/api/omni.isaac.gym

# Cartpole مثال چلائیں
python cartpole.py
```

پیرالل میں 2048 کارٹ پولز ٹریننگ کرتا ہے!

## ہفتہ 9 عملی پروجیکٹ

**کام**: سنتھیٹک ڈیٹاسیٹ بنائیں اور سادہ ماڈل ٹریننگ کریں

**حصہ 1: سنتھیٹک ڈیٹاسیٹ (50 پوائنٹس)**
- سین: 3-6 رنگین کیوبز کے ساتھ میز
- رینڈمائزیشن: لائٹنگ (3 ذرائع)، کیوب پوزیشنز، کیوب رنگ
- 2000 تصاویر تیار کریں (640x480)
- اینوٹیشنز: کوکو فارمیٹ میں باؤنڈنگ باکسز
- ڈیٹاسیٹ کو ڈسک پر محفوظ کریں

**حصہ 2: ماڈل ٹریننگ (50 پوائنٹس)**
- سنتھیٹک ڈیٹا پر یولو وی 8 یا فاسٹر آر-سی این این ٹریننگ کریں
- 200-تصویر ویلیڈیشن سیٹ پر ایویلیوایٹ کریں
- ایم اے پی (مین ایوریج پریسیژن) رپورٹ کریں
- سم ٹو ریئل گیپ کا جائزہ لینے کے لیے حقیقی تصاویر پر جانچ کریں (اگر دستیاب ہوں)

**ڈیلیور ایبلز:**
- ڈیٹاسیٹ جنریشن کے لیے ریپلیکیٹر اسکرپٹ
- ٹریننگ اسکرپٹ اور لاگز
- ٹرینڈ ماڈل ویٹس
- میٹرکس کے ساتھ ایویلیوایشن رپورٹ

**بونس (+20 پوائنٹس):**
- کسٹم ڈومین رینڈمائزیشن نافذ کریں (ڈسٹریکٹر آبجیکٹس، کیمرا نوائز)
- سادہ مینیپولیشن ٹاسک کے لیے آر ایل پالیسی ٹریننگ کریں

## وسائل

- [ریپلیکیٹر دستاویزات](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)
- [آئزک جم دستاویزات](https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/index.html)
- [سنتھیٹک ڈیٹا جنریشن گائیڈ](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)
- [ڈومین رینڈمائزیشن پیپر](https://arxiv.org/abs/1703.06907)
- [آئزک جم بینچ مارک](https://leggedrobotics.github.io/rl-games/)

## اگلے قدم

بہترین کام! اب آپ سنتھیٹک ڈیٹا جنریشن اور پیرالل آر ایل ٹریننگ سمجھتے ہیں۔

اگلا ہفتہ: [ہفتہ 10: سم ٹو ریئل ٹرانسفر اور باب 3 پروجیکٹ](week-10.md)

ہم سم ٹو ریئل گیپ سے نمٹیں گے اور ایک جامع آئزک سم پروجیکٹ مکمل کریں گے!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز ملٹیپل چوائس ہے، خودکار طور پر اسکور ہوتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 9 کوئز لیں →](/quiz?week=9)**
