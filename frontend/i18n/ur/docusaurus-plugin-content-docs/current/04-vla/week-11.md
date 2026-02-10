# ہفتہ 11: ویژن-لینگویج-ایکشن بنیادیں

## خلاصہ

حتمی ماڈیول میں خوش آمدید! یہ ہفتہ ویژن-لینگویج-ایکشن (وی ایل اے) ماڈلز متعارف کراتا ہے، جو فزیکل اے آئی کی جدید ترین صورت ہے۔ آپ سیکھیں گے کہ کیسے ملٹی موڈل ٹرانسفارمرز روبوٹس کو بصری مناظر سمجھنے، قدرتی زبان کے احکامات کی تشریح کرنے، اور مناسب ایکشنز پیدا کرنے کے قابل بناتے ہیں۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- وی ایل اے ماڈل آرکیٹیکچر اور اس کے اجزاء کو سمجھنا
- وضاحت کرنا کہ ویژن، لینگویج، اور ایکشن موڈالیٹیز کو کیسے یکجا کیا جاتا ہے
- پہلے سے تربیت یافتہ ماڈلز کے ساتھ ایک سادہ وی ایل اے پائپ لائن نافذ کرنا
- مخصوص روبوٹک کاموں کے لیے وی ایل اے ماڈلز کو فائن ٹیون کرنا
- وی ایل اے ماڈلز کو آر او ایس 2 اور سمیولیٹرز کے ساتھ انٹیگریٹ کرنا
- وی ایل اے ماڈل کی کارکردگی کا مقداری طور پر جائزہ لینا

## وی ایل اے ماڈلز کیا ہیں؟

**ویژن-لینگویج-ایکشن (وی ایل اے)** ماڈلز ملٹی موڈل ٹرانسفارمرز ہیں جو یہ میپ کرتے ہیں:
- **ویژن** (کیمرہ تصاویر، ڈیپتھ، پوائنٹ کلاؤڈز)
- **لینگویج** (قدرتی زبان کی ہدایات)
→ **ایکشنز** (روبوٹ موٹر کمانڈز، ٹریجیکٹریز)

### وی ایل اے ماڈلز کیوں؟

**روایتی روبوٹکس پائپ لائن:**
```
Perception → State Estimation → Planning → Control
(علیحدہ modules، ہاتھ سے بنائے گئے interfaces)
```

**وی ایل اے اپروچ:**
```
Vision + Language → Transformer → Actions
(end-to-end learning، unified representation)
```

**فوائد:**
- **عمومیت**: واحد ماڈل متنوع کاموں کو سنبھالتا ہے
- **قدرتی انٹریکشن**: زبان کی ہدایات قبول کرتا ہے
- **ٹرانسفر لرننگ**: انٹرنیٹ اسکیل ویژن-لینگویج ڈیٹا سے فائدہ اٹھاتا ہے
- **سادگی**: ہاتھ سے بنائے گئے پرسیپشن/پلاننگ ماڈیولز کی ضرورت نہیں

### حقیقی دنیا کے وی ایل اے سسٹمز

| ماڈل | آرگنائزیشن | اہم کامیابی |
|-------|-------------|-----------------|
| **آر ٹی-1** | گوگل ڈیپ مائنڈ | حقیقی روبوٹس پر 700 کام |
| **آر ٹی-2** | گوگل ڈیپ مائنڈ | ویژن-لینگویج ماڈل → ایکشنز |
| **پام-ای** | گوگل | 540 بی پیرامیٹر ایمباڈیڈ اے آئی |
| **روبو فلیمنگو** | اوپن سورس | اوپن-ویٹس وی ایل اے |
| **اوپن وی ایل اے** | اوپن سورس | 7 بی پیرامیٹر اوپن ماڈل |

## وی ایل اے آرکیٹیکچر

### اعلیٰ سطحی ڈھانچہ

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

### جزو کی تفصیلات

#### 1. ویژن انکوڈر

تصاویر کو ٹوکن ایمبیڈنگز میں تبدیل کرتا ہے:

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

**کلیدی آرکیٹیکچرز:**
- **وی آئی ٹی (ویژن ٹرانسفارمر)**: پیچ بیسڈ امیج انکوڈنگ
- **کلپ**: ویژن-لینگویج کنٹراسٹیو لرننگ
- **ڈینو وی 2**: سیلف-سپروائزڈ ویژن فیچرز
- **آر 3 ایم**: روبوٹ-سپیسیفک ویژن ریپریزنٹیشنز

#### 2. لینگویج انکوڈر

متن کی ہدایات کو انکوڈ کرتا ہے:

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

**عام انکوڈرز:**
- **ٹی 5**: ٹیکسٹ ٹو ٹیکسٹ ٹرانسفارمر
- **برٹ**: بائی ڈائریکشنل لینگویج ماڈل
- **لاما/جی پی ٹی**: لارج لینگویج ماڈلز
- **کلپ ٹیکسٹ**: ویژن کے ساتھ الائنڈ

#### 3. ٹرانسفارمر کور

ویژن اور لینگویج کو فیوز کرتا ہے، ایکشن ریپریزنٹیشنز آؤٹ پٹ کرتا ہے:

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

#### 4. ایکشن ڈیکوڈر

فیوزڈ فیچرز سے روبوٹ ایکشنز کی پیشین گوئی کرتا ہے:

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

### مکمل وی ایل اے ماڈل

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

## وی ایل اے ماڈلز کی تربیت

### ڈیٹاسیٹ کے تقاضے

**ڈیٹا کی شکل:**
```python
{
    "image": PIL.Image (H, W, 3),
    "instruction": str,
    "actions": np.array (action_horizon, action_dim),
    "success": bool
}
```

**مثال کا ٹریجیکٹری:**
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

### تربیتی لوپ

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

### ڈیٹا اکٹھا کرنے کی حکمت عملی

#### 1. ٹیلی آپریشن

انسان وی آر/کی بورڈ کے ذریعے کاموں کا مظاہرہ کرتا ہے:
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

#### 2. سمیولیشن (آئزک سم)

خودکار طور پر ڈیٹا پیدا کریں:
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

#### 3. کو-ٹریننگ (سم + ریئل)

سمیولیٹڈ اور حقیقی ڈیٹا کو ملائیں:
```python
# 90% sim, 10% real
dataset = combine_datasets(
    sim_data=collect_sim_data(sim_env, num_episodes=9000),
    real_data=collect_teleop_data(real_robot, num_episodes=1000),
    real_weight=10.0  # Upweight real data
)
```

## پہلے سے تربیت یافتہ وی ایل اے ماڈلز کا استعمال

### اوپن وی ایل اے (7 بی پیرامیٹرز)

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

### آر ٹی-2 (بذریعہ ہگنگ فیس)

```python
# RT-2 is not publicly released, but similar models:
from transformers import RT2Model  # Hypothetical

model = RT2Model.from_pretrained("google/rt-2-base")

# Run inference
action = model.predict(image=image, instruction=instruction)
robot.move_to(action)
```

## آر او ایس 2 کے ساتھ وی ایل اے کو انٹیگریٹ کرنا

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

## ہفتہ 11 عملی مشق

**کام**: روبوٹ مینیپولیشن کے لیے پہلے سے تربیت یافتہ ویژن انکوڈر کو فائن ٹیون کریں

**تقاضے:**
1. آر ٹی-1 یا اوپن وی ایل اے ڈیمو ڈیٹاسیٹ ڈاؤن لوڈ کریں (100 ٹریجیکٹریز)
2. روبوٹ تصاویر پر کلپ ویژن انکوڈر کو فائن ٹیون کریں
3. سادہ ایکشن پریڈکشن ہیڈ (ایم ایل پی) نافذ کریں
4. 10 ایپوکس کے لیے تربیت دیں
5. ایکشن پریڈکشن کی درستگی کا جائزہ لیں
6. پیشین گوئی شدہ بمقابلہ گراؤنڈ-ٹروتھ ایکشنز کو ویژولائز کریں

**بونس:**
- لینگویج کنڈیشننگ شامل کریں
- آئزک سم انوائرنمنٹ پر ٹیسٹ کریں
- ایکشن چنکنگ نافذ کریں (10 قدم آگے کی پیشین گوئی)

## وسائل

- [آر ٹی-1 پیپر](https://arxiv.org/abs/2212.06817) - روبوٹکس ٹرانسفارمر فار ریئل-ورلڈ کنٹرول
- [آر ٹی-2 پیپر](https://arxiv.org/abs/2307.15818) - ویژن-لینگویج-ایکشن ماڈلز
- [اوپن وی ایل اے](https://openvla.github.io/) - اوپن سورس 7 بی وی ایل اے ماڈل
- [روبو فلیمنگو](https://roboflamingo.github.io/) - اوپن-ویٹس وی ایل اے
- [پام-ای پیپر](https://arxiv.org/abs/2303.03378) - ایمباڈیڈ ملٹی موڈل لینگویج ماڈل

## اگلے قدم

شاندار کام! اب آپ وی ایل اے آرکیٹیکچر کو سمجھتے ہیں اور پہلے سے تربیت یافتہ ماڈلز کے ساتھ کام کر سکتے ہیں۔

اگلا ہفتہ: [ہفتہ 12: اعلیٰ درجے کے وی ایل اے ایپلیکیشنز](week-12.md)

ہم ایکشن چنکنگ، ملٹی ٹاسک لرننگ، اور حقیقی روبوٹس پر وی ایل اے ماڈلز کی تعیناتی کو تلاش کریں گے!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو ٹیسٹ کریں! کوئز ملٹیپل چوائس ہے، خودکار طور پر اسکورڈ ہے، اور آپ کے پاس 2 اٹیمپٹس ہیں۔

**[ہفتہ 11 کوئز لیں →](/quiz?week=11)**
