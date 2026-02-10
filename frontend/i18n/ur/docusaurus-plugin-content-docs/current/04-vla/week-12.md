# ہفتہ 12: اعلیٰ درجے کے وی ایل اے ایپلیکیشنز

## خلاصہ

یہ ہفتہ اعلیٰ درجے کی وی ایل اے تکنیکوں کو تلاش کرتا ہے بشمول ایکشن چنکنگ، ملٹی ٹاسک لرننگ، فیو-شاٹ ایڈاپٹیشن، اور حقیقی دنیا میں تعیناتی کی حکمت عملی۔ آپ سیکھیں گے کہ وی ایل اے ماڈلز کو زیادہ مضبوط، موثر اور عمومی کیسے بنایا جائے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- وقتی طور پر مستقل پیشین گوئیوں کے لیے ایکشن چنکنگ نافذ کرنا
- متنوع کاموں کو سنبھالنے والے ملٹی ٹاسک وی ایل اے ماڈلز کی تربیت کرنا
- تیز رفتار کام کی موافقت کے لیے فیو-شاٹ لرننگ استعمال کرنا
- اعلیٰ معیار کے ایکشن جنریشن کے لیے ڈفیوژن پالیسیز استعمال کرنا
- وسائل محدود روبوٹس پر وی ایل اے ماڈلز کو تعینات کرنا
- ناکامی کے معاملات سنبھالنا اور ریکوری رویوں کو نافذ کرنا

## ایکشن چنکنگ

**مسئلہ**: ایک ٹائم اسٹیپ پر ایکشنز کی پیشین گوئی کرنا مایوپک ہے اور جٹری رویے کا باعث بنتا ہے۔

**حل**: متعدد ٹائم اسٹیپس کا احاطہ کرنے والے ایکشن سیکوئنسز (چنکس) کی پیشین گوئی کریں۔

### نفاذ

```python
class ActionChunkingVLA(nn.Module):
    """VLA with action chunking."""

    def __init__(self, action_dim=7, chunk_size=10):
        super().__init__()

        self.chunk_size = chunk_size
        self.action_dim = action_dim

        # Vision-language encoder (same as before)
        self.vl_encoder = VLATransformer()

        # Action decoder predicts chunk_size actions
        self.action_decoder = nn.Sequential(
            nn.Linear(768, 512),
            nn.ReLU(),
            nn.Linear(512, chunk_size * action_dim)
        )

    def forward(self, image, instruction):
        """
        Returns:
            action_chunk: (batch, chunk_size, action_dim)
        """
        # Encode vision and language
        vl_features = self.vl_encoder(image, instruction)

        # Global pool
        pooled = vl_features.mean(dim=1)  # (batch, 768)

        # Predict action chunk
        flat_actions = self.action_decoder(pooled)  # (batch, chunk_size * action_dim)
        action_chunk = flat_actions.view(-1, self.chunk_size, self.action_dim)

        return action_chunk
```

### ایکشن چنکنگ کے ساتھ عملدرآمد

```python
class RobotController:
    """Execute actions with chunking."""

    def __init__(self, model, chunk_size=10):
        self.model = model
        self.chunk_size = chunk_size
        self.action_buffer = []
        self.buffer_index = 0

    def get_action(self, image, instruction):
        """Get next action from buffer or predict new chunk."""

        # Predict new chunk if buffer empty
        if self.buffer_index >= len(self.action_buffer):
            action_chunk = self.model(image, instruction)  # (1, chunk_size, 7)
            self.action_buffer = action_chunk[0].cpu().numpy()  # (chunk_size, 7)
            self.buffer_index = 0

        # Get next action from buffer
        action = self.action_buffer[self.buffer_index]
        self.buffer_index += 1

        return action
```

**فوائد:**
- ہموار ٹریجیکٹریز
- بہتر وقتی استقامت
- تیز انفرنس (1 فارورڈ پاس → 10 ایکشنز)

**ٹریڈ آفس:**
- تبدیلیوں کے لیے کم ری ایکٹو
- اگر ماحول تبدیل ہو تو دوبارہ منصوبہ بندی کی ضرورت

### اڈاپٹو ری-پلاننگ

```python
def execute_with_replanning(robot, model, image, instruction):
    """Execute action chunk with dynamic replanning."""

    chunk_size = 10
    replan_threshold = 0.1  # Replan if deviation > 10cm

    while not task_complete:
        # Get action chunk
        action_chunk = model(image, instruction)

        for i, action in enumerate(action_chunk):
            # Execute action
            robot.execute(action)

            # Check if need to replan
            current_image = robot.get_camera_image()
            predicted_state = predict_state_after_action(action)
            actual_state = get_robot_state()

            deviation = np.linalg.norm(predicted_state - actual_state)

            if deviation > replan_threshold:
                print(f"Deviation {deviation:.3f} > threshold, replanning...")
                break  # Exit inner loop, generate new chunk

            time.sleep(0.1)  # 10 Hz control
```

## ملٹی ٹاسک لرننگ

**ہدف**: واحد وی ایل اے ماڈل جو سینکڑوں مختلف کاموں کو سنبھالے۔

### ٹاسک کنڈیشننگ

```python
class MultiTaskVLA(nn.Module):
    """VLA for multiple tasks with task embeddings."""

    def __init__(self, num_tasks=500, action_dim=7):
        super().__init__()

        # Task embedding lookup table
        self.task_embeddings = nn.Embedding(num_tasks, 768)

        # Vision-language encoder
        self.vl_encoder = VLATransformer()

        # Task-conditioned action decoder
        self.action_decoder = nn.Sequential(
            nn.Linear(768 * 2, 512),  # VL features + task embedding
            nn.ReLU(),
            nn.Linear(512, action_dim)
        )

    def forward(self, image, instruction, task_id):
        """
        Args:
            task_id: int, task identifier (0 to num_tasks-1)
        """
        # Encode vision-language
        vl_features = self.vl_encoder(image, instruction).mean(dim=1)  # (batch, 768)

        # Get task embedding
        task_emb = self.task_embeddings(task_id)  # (batch, 768)

        # Concatenate
        combined = torch.cat([vl_features, task_emb], dim=1)  # (batch, 1536)

        # Predict action
        action = self.action_decoder(combined)

        return action
```

### ملٹی ٹاسک ڈیٹاسیٹ کی تربیت

```python
# Dataset with diverse tasks
multi_task_data = [
    {"task_id": 0, "instruction": "pick red cube", "image": img1, "action": act1},
    {"task_id": 1, "instruction": "push blue box", "image": img2, "action": act2},
    {"task_id": 2, "instruction": "open drawer", "image": img3, "action": act3},
    # ... 500 tasks
]

# Training loop
for batch in dataloader:
    images = batch["image"]
    instructions = batch["instruction"]
    task_ids = batch["task_id"]
    target_actions = batch["action"]

    # Forward pass
    predicted_actions = model(images, instructions, task_ids)

    # Loss
    loss = criterion(predicted_actions, target_actions)

    # Backward
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
```

### ٹاسک-فیملی گروپنگ

```python
# Group related tasks for better generalization
task_families = {
    "pick": [0, 1, 2, ...],       # Pick different objects
    "place": [50, 51, 52, ...],   # Place in different locations
    "push": [100, 101, 102, ...], # Push various objects
    "open": [150, 151, 152, ...], # Open doors/drawers
}

# Hierarchical task embedding
class HierarchicalTaskEmbedding(nn.Module):
    def __init__(self, num_families=10, num_tasks_per_family=50):
        super().__init__()

        self.family_embeddings = nn.Embedding(num_families, 384)
        self.task_embeddings = nn.Embedding(num_tasks_per_family, 384)

    def forward(self, family_id, task_id):
        family_emb = self.family_embeddings(family_id)
        task_emb = self.task_embeddings(task_id)
        return torch.cat([family_emb, task_emb], dim=1)  # (768,)
```

## فیو-شاٹ ایڈاپٹیشن

**منظر نامہ**: صرف 5-10 مظاہروں کے ساتھ نیا کام۔

### میٹا-لرننگ (ایم اے ایم ایل)

```python
import learn2learn as l2l

def meta_train_vla(model, meta_train_tasks, num_iterations=10000):
    """Meta-train VLA for few-shot adaptation."""

    meta_model = l2l.algorithms.MAML(model, lr=1e-3, first_order=False)
    meta_optimizer = torch.optim.Adam(meta_model.parameters(), lr=1e-4)

    for iteration in range(num_iterations):
        meta_loss = 0.0

        # Sample batch of tasks
        tasks = random.sample(meta_train_tasks, k=8)

        for task in tasks:
            # Clone model for inner loop
            learner = meta_model.clone()

            # Inner loop: adapt to task with few examples
            support_data = task.sample_support(k=5)  # 5-shot
            for step in range(5):  # 5 inner steps
                pred = learner(support_data["image"], support_data["instruction"])
                loss = criterion(pred, support_data["action"])
                learner.adapt(loss)

            # Outer loop: evaluate on query set
            query_data = task.sample_query(k=10)
            pred = learner(query_data["image"], query_data["instruction"])
            loss = criterion(pred, query_data["action"])

            meta_loss += loss

        # Meta-update
        meta_optimizer.zero_grad()
        meta_loss.backward()
        meta_optimizer.step()

    return meta_model
```

### تیز رفتار فائن ٹیوننگ

```python
def few_shot_finetune(pretrained_model, new_task_data, num_shots=10):
    """Fine-tune pre-trained VLA on new task with few examples."""

    # Freeze early layers, only train action head
    for param in pretrained_model.vl_encoder.parameters():
        param.requires_grad = False

    # Only train action decoder
    for param in pretrained_model.action_decoder.parameters():
        param.requires_grad = True

    # Use small learning rate
    optimizer = torch.optim.Adam(
        pretrained_model.action_decoder.parameters(),
        lr=1e-5
    )

    # Train on few examples with high data augmentation
    for epoch in range(100):  # Many epochs on few examples
        for sample in new_task_data[:num_shots]:  # Only k shots
            # Augment data
            augmented_image = augment(sample["image"])

            # Forward
            pred = pretrained_model(augmented_image, sample["instruction"])
            loss = criterion(pred, sample["action"])

            # Backward
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

    return pretrained_model
```

## ڈفیوژن پالیسیز

**خیال**: ایکشنز پیدا کرنے کے لیے ڈفیوژن ماڈلز استعمال کریں (جیسے ایکشنز کے لیے ڈال-ای)۔

### ڈفیوژن ایکشن ڈیکوڈر

```python
import torch
from diffusers import DDPMScheduler

class DiffusionActionDecoder(nn.Module):
    """Generate actions via iterative denoising."""

    def __init__(self, action_dim=7, num_diffusion_steps=10):
        super().__init__()

        self.action_dim = action_dim
        self.num_steps = num_diffusion_steps

        # Noise predictor
        self.noise_predictor = nn.Sequential(
            nn.Linear(768 + action_dim + 1, 512),  # VL features + noisy action + timestep
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, action_dim)
        )

        # Diffusion scheduler
        self.scheduler = DDPMScheduler(num_train_timesteps=num_diffusion_steps)

    def forward(self, vl_features, num_samples=1):
        """
        Generate actions via denoising.

        Args:
            vl_features: (batch, 768) from vision-language encoder
            num_samples: Number of action samples to generate
        Returns:
            actions: (batch, num_samples, action_dim)
        """
        batch_size = vl_features.shape[0]

        # Start from pure noise
        actions = torch.randn(batch_size, num_samples, self.action_dim, device=vl_features.device)

        # Iteratively denoise
        for t in reversed(range(self.num_steps)):
            timestep = torch.full((batch_size,), t, device=vl_features.device)

            # Predict noise
            for i in range(num_samples):
                action_sample = actions[:, i, :]  # (batch, action_dim)

                # Concatenate VL features, noisy action, timestep
                input_tensor = torch.cat([
                    vl_features,
                    action_sample,
                    timestep.unsqueeze(1).float()
                ], dim=1)

                # Predict noise to remove
                predicted_noise = self.noise_predictor(input_tensor)

                # Denoise action
                actions[:, i, :] = self.scheduler.step(
                    predicted_noise, t, action_sample
                ).prev_sample

        return actions
```

**فوائد:**
- ملٹی-موڈل ایکشن ڈسٹری بیوشنز (متعدد درست ایکشنز)
- ابہام کو بہتر سنبھالتا ہے
- ہموار ایکشن ٹریجیکٹریز

**تربیت:**
```python
def train_diffusion_policy(model, dataloader):
    """Train diffusion policy."""
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

    for batch in dataloader:
        vl_features = model.vl_encoder(batch["image"], batch["instruction"])
        target_actions = batch["action"]

        # Sample random timestep
        timesteps = torch.randint(0, model.num_steps, (batch_size,))

        # Add noise to target actions
        noise = torch.randn_like(target_actions)
        noisy_actions = model.scheduler.add_noise(target_actions, noise, timesteps)

        # Predict noise
        predicted_noise = model.noise_predictor(
            torch.cat([vl_features, noisy_actions, timesteps.unsqueeze(1)], dim=1)
        )

        # Loss: MSE between predicted and actual noise
        loss = nn.MSELoss()(predicted_noise, noise)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

## حقیقی دنیا میں تعیناتی

### ماڈل کی بہتری

#### 1. کوانٹائزیشن (ماڈل کا سائز کم کریں)

```python
import torch.quantization

# Post-training quantization
model_fp32 = VLAModel()
model_fp32.load_state_dict(torch.load("vla_model.pth"))
model_fp32.eval()

# Quantize to INT8
model_int8 = torch.quantization.quantize_dynamic(
    model_fp32,
    {nn.Linear, nn.Conv2d},
    dtype=torch.qint8
)

# Model size: 7GB → 1.8GB (4x smaller)
# Inference speed: 2x faster on CPU

torch.save(model_int8.state_dict(), "vla_model_int8.pth")
```

#### 2. نالج ڈسٹلیشن

```python
def distill_vla(large_teacher, small_student, dataloader):
    """Distill large VLA into small model."""

    teacher = large_teacher.eval()
    student = small_student.train()

    optimizer = torch.optim.Adam(student.parameters(), lr=1e-4)
    temperature = 3.0  # Softening factor

    for batch in dataloader:
        images = batch["image"]
        instructions = batch["instruction"]

        # Teacher predictions (frozen)
        with torch.no_grad():
            teacher_actions = teacher(images, instructions)

        # Student predictions
        student_actions = student(images, instructions)

        # Distillation loss (match teacher)
        distill_loss = nn.MSELoss()(student_actions, teacher_actions)

        # Ground-truth loss (match labels)
        gt_loss = nn.MSELoss()(student_actions, batch["action"])

        # Combined loss
        loss = 0.5 * distill_loss + 0.5 * gt_loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return student
```

### ناکامی کی تشخیص اور بحالی

```python
class SafeVLAController:
    """VLA with safety checks and failure recovery."""

    def __init__(self, model):
        self.model = model
        self.confidence_threshold = 0.8
        self.collision_threshold = 0.05  # 5cm
        self.max_retries = 3

    def execute_with_safety(self, image, instruction, robot):
        """Execute action with safety checks."""

        retries = 0

        while retries < self.max_retries:
            # Predict action
            action, confidence = self.model.predict_with_confidence(image, instruction)

            # Safety check 1: Low confidence
            if confidence < self.confidence_threshold:
                print(f"Low confidence ({confidence:.2f}), requesting human help...")
                return self.request_human_intervention(robot, instruction)

            # Safety check 2: Collision prediction
            if self.will_collide(robot, action):
                print("Collision detected, trying alternative action...")
                retries += 1
                continue

            # Execute action
            success = robot.execute(action)

            if success:
                return True

            # Failed, retry
            print(f"Execution failed, retry {retries+1}/{self.max_retries}")
            retries += 1

        # All retries exhausted
        return self.execute_recovery_behavior(robot)

    def will_collide(self, robot, action):
        """Check if action will cause collision."""
        predicted_position = robot.forward_kinematics(action)
        obstacles = robot.get_obstacle_map()

        # Check distance to nearest obstacle
        min_distance = compute_min_distance(predicted_position, obstacles)
        return min_distance < self.collision_threshold

    def execute_recovery_behavior(self, robot):
        """Fallback recovery behavior."""
        # Move to safe home position
        robot.move_to_home()
        return False
```

## ہفتہ 12 عملی پروجیکٹ

**کام**: وی ایل اے ویریئنٹس کو نافذ کریں اور موازنہ کریں

**حصہ 1: ایکشن چنکنگ (40 پوائنٹس)**
- 10-اسٹیپ ایکشن چنکس کی پیشین گوئی کے لیے بیس لائن وی ایل اے میں ترمیم کریں
- پک اینڈ پلیس ڈیٹاسیٹ پر تربیت دیں
- سنگل-اسٹیپ پریڈکشن کے مقابلے ٹریجیکٹری سموتھنیس کا موازنہ کریں
- رپورٹ: کامیابی کی شرح، جرکینیس میٹرک

**حصہ 2: ملٹی ٹاسک لرننگ (40 پوائنٹس)**
- 5 مختلف کاموں کے ساتھ ڈیٹاسیٹ بنائیں
- ٹاسک کنڈیشننگ کے ساتھ ملٹی ٹاسک وی ایل اے کی تربیت کریں
- 6ویں کام میں زیرو-شاٹ ٹرانسفر کا جائزہ لیں
- رپورٹ: فی کام کارکردگی، جنرلائزیشن گیپ

**حصہ 3: ڈیپلائمنٹ (20 پوائنٹس)**
- ماڈل کو آئی این ٹی 8 میں کوانٹائز کریں
- انفرنس لیٹنسی ناپیں (پہلے/بعد میں)
- سادہ فیلیئر ڈیٹیکشن نافذ کریں
- ڈیمو ویڈیو ریئل ٹائم عملدرآمد دکھاتی ہوئی

**بونس (+20 پوائنٹس):**
- ڈفیوژن پالیسی نافذ کریں
- ایم اے ایم ایل کے ساتھ میٹا-لرننگ
- جیٹسن نینو یا اسی طرح کے ایج ڈیوائس پر تعینات کریں

## وسائل

- [ڈفیوژن پالیسی پیپر](https://arxiv.org/abs/2303.04137)
- [آر ٹی-1: روبوٹکس ٹرانسفارمر](https://arxiv.org/abs/2212.06817)
- [ایم اے ایم ایل فار آر ایل](https://arxiv.org/abs/1703.03400)
- [ایکشن چنکنگ پیپر](https://arxiv.org/abs/2304.13705)
- [کوانٹائزیشن گائیڈ](https://pytorch.org/docs/stable/quantization.html)

## اگلے قدم

حیرت انگیز پیش رفت! آپ نے اعلیٰ درجے کی وی ایل اے تکنیکوں میں مہارت حاصل کر لی ہے۔

اگلا ہفتہ: [ہفتہ 13: کیپ سٹون پروجیکٹ](week-13.md)

آخری ہفتہ! آپ ایک مکمل فزیکل اے آئی سسٹم ڈیزائن اور نافذ کریں گے جو آپ نے سیکھا ہے اسے یکجا کرتے ہوئے!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو ٹیسٹ کریں! کوئز ملٹیپل چوائس ہے، خودکار طور پر اسکورڈ ہے، اور آپ کے پاس 2 اٹیمپٹس ہیں۔

**[ہفتہ 12 کوئز لیں →](/quiz?week=12)**
