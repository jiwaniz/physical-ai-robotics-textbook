# ہفتہ 10: سم ٹو ریئل ٹرانسفر اور باب 3 پروجیکٹ

## جائزہ

باب 3 کا یہ آخری ہفتہ سم ٹو ریئل ٹرانسفر کے نازک چیلنج سے نمٹتا ہے: سمیولیشن میں ٹرینڈ پالیسیز اور ماڈلز کو حقیقی روبوٹس پر کام کرنا۔ آپ ثابت شدہ تکنیکیں سیکھیں گے، ٹرانسفر حکمت عملی نافذ کریں گے، اور ایک جامع آئزک سم پروجیکٹ مکمل کریں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- سم ٹو ریئل گیپ کی وجوہات سمجھیں
- منظم طریقے سے ڈومین رینڈمائزیشن تکنیکیں لگائیں
- فزکس کیلیبریشن کے لیے سسٹم آئیڈینٹیفکیشن نافذ کریں
- سم ٹو ریئل ٹرانسفر کے بہترین طریقے استعمال کریں
- ٹرانسفر کی کامیابی کا مقداری جائزہ لیں
- باب 3 کا اسیسمنٹ پروجیکٹ مکمل کریں
- سمیولیٹڈ پالیسیز کو حقیقی ہارڈویئر پر ڈیپلائے کریں (تصوراتی طور پر)

## سم ٹو ریئل گیپ

### سم ٹو ریئل گیپ کیا ہے؟

**تعریف**: سمیولیشن سے حقیقی دنیا میں پالیسیز/ماڈلز منتقل کرتے وقت کارکردگی میں کمی۔

**مثال:**
- **سمیولیشن میں**: روبوٹ 95% وقت اشیاء کو گراسپ کرتا ہے
- **حقیقی روبوٹ پر**: روبوٹ 40% وقت اشیاء کو گراسپ کرتا ہے
- **گیپ**: 55% کارکردگی کا نقصان

### بنیادی وجوہات

#### 1. فزکس تضادات

| خاصیت | سمیولیشن | حقیقت |
|----------|------------|---------|
| **فرکشن** | مستقل، آسان کیا ہوا | متغیر، پیچیدہ |
| **کانٹیکٹ** | پینیٹریشن پر مبنی | ڈیفارمیشن، سلپ |
| **ڈائنامکس** | ڈیٹرمنسٹک | اسٹوکیسٹک |
| **تاخیر** | کوئی نہیں | موٹر لیگ، سینسر لیٹنسی |
| **نوائز** | گاوسیئن (اگر شامل ہو) | نان-گاوسیئن، وقت سے منسلک |

#### 2. بصری تضادات

- **رینڈرنگ**: کامل رے ٹریسنگ بمقابلہ حقیقی کیمرا نوائز/بلر
- **لائٹنگ**: کنٹرول شدہ بمقابلہ متغیر محیطی روشنی
- **ٹیکسچرز**: صاف 3ڈی ماڈلز بمقابلہ گھسے ہوئے/گندے حقیقی اشیاء
- **اوکلوژن**: کامل بمقابلہ جزوی سینسر کوریج

#### 3. ماڈلنگ کی غلطیاں

- **آسان کی ہوئی جیومیٹری**: سی اے ڈی ماڈلز بمقابلہ مینوفیکچرڈ ٹالرینسز
- **ماس/انرشیا ایررز**: تخمینہ شدہ بمقابلہ حقیقی خصوصیات
- **سینسر ماڈلز**: مثالی بمقابلہ حقیقی سینسر خصوصیات
- **ایکچوایشن**: کامل موٹرز بمقابلہ بیک لیش/کمپلائنس

## گیپ کو پاٹنا: ثابت شدہ تکنیکیں

### 1. ڈومین رینڈمائزیشن (ڈی آر)

**خیال**: سمیولیشن پیرامیٹرز کو رینڈمائز کریں تاکہ حقیقی دنیا صرف ایک اور ویریایشن ہو۔

**رینڈمائز کرنے کے لیے پیرامیٹرز:**

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

**ڈی آر کے ساتھ ٹریننگ:**

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

### 2. سسٹم آئیڈینٹیفکیشن

**ہدف**: حقیقی روبوٹ خصوصیات کی پیمائش کریں اور سمیولیشن کو میچ کریں۔

**مرحلہ 1: فرکشن کی شناخت**

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

**مرحلہ 2: سمیولیشن کو اپ ڈیٹ کریں**

```xml
<!-- شناخت شدہ parameters کے ساتھ URDF/USD کو اپ ڈیٹ کریں -->
<gazebo reference="link">
  <mu1>0.68</mu1>  <!-- شناخت شدہ friction -->
  <mu2>0.68</mu2>
</gazebo>
```

### 3. پریویلجڈ لرننگ + اڈاپٹیشن

**خیال**: کامل سم معلومات کے ساتھ ٹریننگ کریں، پھر ڈیپلائمنٹ پر اڈاپٹ کریں۔

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

**پریویلجڈ معلومات کی مثالیں:**
- حقیقی آبجیکٹ ماس، فرکشن کوایفیشنٹس
- گراؤنڈ-ٹروتھ آبجیکٹ پوزز (بمقابلہ نوائزی پرسیپشن)
- مستقبل کا ٹریجیکٹری (پیشن گوئی کے کاموں کے لیے)
- پوشیدہ سٹیٹ (جوائنٹ فورسز، کانٹیکٹ پوائنٹس)

### 4. حقیقی ڈیٹا پر فائن ٹیوننگ

**حکمت عملی**: سم میں ٹریننگ کریں، چھوٹے حقیقی ڈیٹاسیٹ کے ساتھ فائن ٹیون کریں۔

```python
# مرحلہ 1: Simulation میں Pre-train کریں (لاکھوں samples)
policy = train_in_simulation(num_steps=10_000_000)

# مرحلہ 2: حقیقی data جمع کریں (سیکڑوں samples)
real_data = collect_real_robot_data(num_episodes=100)

# مرحلہ 3: حقیقی data پر Fine-tune کریں
policy = fine_tune(policy, real_data, num_epochs=50, lr=1e-5)
```

**بہترین طریقے:**
- کیٹاسٹروفک فرگیٹنگ سے بچنے کے لیے کم لرننگ ریٹ استعمال کریں
- سمیولیشن سے 90% ٹریننگ ڈیٹا رکھیں
- حقیقی ڈیٹا کو فیلیئر موڈز پر فوکس کریں

### 5. ریزیجوئل لرننگ

**خیال**: سم پالیسی کے اوپر کریکشن سیکھیں۔

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

**ٹریننگ:**
- حقیقی روبوٹ پر ریزیجوئل نیٹ ورک ٹریننگ کریں
- سم پالیسی کا علم رکھتا ہے، صرف کریکشنز سیکھتا ہے

## مقداری ٹرانسفر ایویلیوایشن

### میٹرکس

**1. کامیابی کی شرح**
```
کامیابی کی شرح = (کامیاب آزمائشیں / کل آزمائشیں) × 100%
```

**2. سم ٹو ریئل پرفارمنس ریشو**
```
ٹرانسفر ریشو = حقیقی کارکردگی / سم کارکردگی
```
- ریشو = 1.0: کامل ٹرانسفر
- ریشو < 0.7: خراب ٹرانسفر (بہتری کی ضرورت)
- ریشو > 0.9: بہترین ٹرانسفر

**3. سیمپل ایفیشنسی**
```
ضروری سیمپلز = 90% سم کارکردگی تک پہنچنے کے لیے حقیقی سیمپلز
```

**4. ٹاسک مخصوص میٹرکس**
- **گراسپنگ**: گراسپ کامیابی کی شرح، گراسپ استحکام
- **نیویگیشن**: ہدف تک پہنچنے کی کامیابی، کولیژن ریٹ
- **مینیپولیشن**: ٹاسک مکمل کرنے کا وقت، پریسیژن

### ایویلیوایشن پروٹوکول

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

## کیس اسٹڈی: گراسپنگ ٹرانسفر

### سمیولیشن سیٹ اپ

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

### حقیقی روبوٹ ڈیپلائمنٹ

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
- کوئی ڈی آر نہیں: 30-40% کامیابی
- ڈی آر کے ساتھ: 70-85% کامیابی
- ڈی آر + فائن ٹیوننگ: 85-95% کامیابی

## باب 3 اسیسمنٹ پروجیکٹ

**کام**: مینیپولیشن ٹاسک کے لیے مکمل آئزک سم پائپ لائن بنائیں

### پروجیکٹ کی ضروریات (100 پوائنٹس)

#### حصہ 1: سین سیٹ اپ (15 پوائنٹس)
- ویئر ہاؤس/فیکٹری انوائرنمنٹ بنائیں
- رکاوٹیں، مختلف لائٹنگ شامل کریں
- مینیپولیشن کے لیے 3+ آبجیکٹ ٹائپس شامل کریں
- کسٹم یو ایس ڈی ماڈلز (بونس: سی اے ڈی سے امپورٹ کریں)

#### حصہ 2: سنتھیٹک ڈیٹا جنریشن (25 پوائنٹس)
- اینوٹیشنز کے ساتھ 5000+ تصاویر تیار کریں
- جامع ڈومین رینڈمائزیشن نافذ کریں:
  - لائٹنگ (3+ ذرائع، مختلف انٹینسیٹی/رنگ)
  - آبجیکٹ پوزز، سائز، ٹیکسچرز
  - کیمرا پیرامیٹرز، نوائز
- کوکو یا کسٹم فارمیٹ میں ایکسپورٹ کریں
- تقسیم: 80% ٹرین، 10% ویل، 10% ٹیسٹ

#### حصہ 3: ویژن ماڈل ٹریننگ (25 پوائنٹس)
- آبجیکٹ ڈیٹیکشن یا سیگمینٹیشن ماڈل ٹریننگ کریں
- صرف سنتھیٹک ڈیٹا استعمال کریں
- ویلیڈیشن سیٹ پر میٹرکس رپورٹ کریں:
  - ڈیٹیکشن کے لیے ایم اے پی@0.5
  - سیگمینٹیشن کے لیے آئی او یو
- ٹیسٹ امیجز پر پریڈکشنز کو ویژولائز کریں

#### حصہ 4: آر ایل پالیسی ٹریننگ (25 پوائنٹس)
- مینیپولیشن ٹاسک کی تعریف کریں (پک اینڈ پلیس، ریچنگ، وغیرہ)
- ڈومین رینڈمائزیشن کے ساتھ انوائرنمنٹ نافذ کریں
- آر ایل الگورتھم کے ساتھ پالیسی ٹریننگ کریں (پی پی او/ایس اے سی/وغیرہ)
- سم میں کامیاب ٹاسک ایگزیکیوشن کا مظاہرہ کریں
- ٹریننگ کروز، کامیابی کی شرح رپورٹ کریں

#### حصہ 5: دستاویزات (10 پوائنٹس)
- سیٹ اپ انسٹرکشنز کے ساتھ ریڈ می
- آرکیٹیکچر ڈایاگرام (سین، سینسرز، الگورتھمز)
- ٹریننگ/ایویلیوایشن رپورٹیں
- ڈیمو ویڈیو (زیادہ سے زیادہ 5 منٹ مکمل پائپ لائن دکھاتے ہوئے)

### ڈیلیور ایبلز

1. **گٹ ہب ریپوزٹری**:
   - تمام سورس کوڈ
   - ڈیٹاسیٹ جنریشن اسکرپٹس
   - ٹریننگ اسکرپٹس
   - ٹرینڈ ماڈلز/پالیسیز

2. **ڈیٹاسیٹ**:
   - تیار شدہ ڈیٹاسیٹ کا لنک (کلاؤڈ اسٹوریج ٹھیک ہے)
   - ریپوزٹری میں 100 سیمپل امیجز

3. **رپورٹ** (پی ڈی ایف، 3-5 صفحات):
   - طریقہ کار
   - ڈومین رینڈمائزیشن حکمت عملی
   - ٹریننگ میٹرکس اور کروز
   - چیلنجز اور حل

4. **ڈیمو ویڈیو**:
   - سین واک تھرو
   - ڈیٹاسیٹ جنریشن پراسیس
   - ماڈل/پالیسی عمل میں
   - مقداری نتائج

### گریڈنگ روبرک

| جزو | پوائنٹس | معیار |
|-----------|--------|----------|
| **سین کوالٹی** | 15 | حقیقت پسندانہ، متنوع، اچھی طرح روشن |
| **ڈیٹاسیٹ کوالٹی** | 15 | بڑا، متنوع، صحیح طریقے سے اینوٹیٹڈ |
| **ڈی آر امپلیمنٹیشن** | 10 | جامع رینڈمائزیشن |
| **ماڈل پرفارمنس** | 15 | درستگی کی حدود کو پورا کرتا ہے |
| **آر ایل پالیسی** | 15 | سم میں ٹاسک کامیابی > 80% |
| **کوڈ کوالٹی** | 10 | صاف، دستاویز شدہ، قابل تکرار |
| **دستاویزات** | 10 | واضح، مکمل، اچھی طرح لکھا ہوا |
| **تخلیقیت** | 10 | نیا طریقہ یا اضافی خصوصیات |

**بونس مواقع** (+زیادہ سے زیادہ 20 پوائنٹس):
- حقیقی روبوٹ پر ڈیپلائے کریں (+15)
- ملٹی ٹاسک لرننگ (+10)
- کسٹم فزکس سمیولیشن (+10)
- ایس ایل اے ایم انٹیگریشن (+10)

### مثال کے پروجیکٹس

**مثال 1: بن پکنگ**
- ٹاسک: بکھرے ہوئے بن سے پارٹس اٹھائیں
- آبجیکٹس: پیچ، نٹ، واشر (مختلف سائز)
- ویژن: ڈیٹیکشن کے لیے یولو وی 8
- پالیسی: پک اینڈ پلیس کے لیے پی پی او

**مثال 2: کوالٹی انسپیکشن**
- ٹاسک: تیار شدہ پارٹس پر خرابیوں کا پتہ لگائیں
- ڈیٹاسیٹ: خرابی اینوٹیشنز کے ساتھ 10 ہزار تصاویر
- ماڈل: سیمینٹک سیگمینٹیشن (یو-نیٹ)
- ڈیپلائمنٹ: ریئل ٹائم انفرنس کے لیے آر او ایس 2 نوڈ

**مثال 3: ویئر ہاؤس نیویگیشن**
- ٹاسک: شیلف تک نیویگیٹ کریں، چیز بازیافت کریں
- انوائرنمنٹ: آئلز کے ساتھ 50x50 میٹر ویئر ہاؤس
- ویژن: رکاوٹوں سے بچنے کے لیے ڈیپتھ کیمرا
- پالیسی: نیویگیشن + گراسپنگ کے لیے ایس اے سی

## ہفتہ 10 عملی مشق

**کام**: ڈی آر حکمت عملیوں کو نافذ اور موازنہ کریں

1. **بغیر** ڈی آر کے بیس لائن پالیسی ٹریننگ کریں
2. صرف لائٹنگ ڈی آر کے ساتھ پالیسی ٹریننگ کریں
3. مکمل ڈی آر کے ساتھ پالیسی ٹریننگ کریں (لائٹنگ + فزکس + ویژن)
4. متغیر ٹیسٹ انوائرنمنٹس میں تینوں کا جائزہ لیں
5. کارکردگی کا موازنہ رپورٹ کریں

**متوقع نتیجہ**: مکمل ڈی آر پالیسی انوائرنمنٹ کی تبدیلیوں کے لیے سب سے زیادہ مضبوط ہونی چاہیے۔

## وسائل

- [سم ٹو ریئل ٹرانسفر سروے](https://arxiv.org/abs/2009.13303)
- [ڈومین رینڈمائزیشن فار ٹرانسفرنگ ڈیپ نیورل نیٹ ورکس](https://arxiv.org/abs/1703.06907)
- [لرننگ ڈیکسٹرس ان-ہینڈ مینیپولیشن](https://arxiv.org/abs/1808.00177) - اوپن اے آئی کا روبکس کیوب
- [این ویڈیا آئزک سم ایگزامپلز](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)
- [سسٹم آئیڈینٹیفکیشن ٹیکنیکس](https://stanford.edu/class/ee363/sysid.pdf)

## باب 3 کا خلاصہ

باب 3 مکمل کرنے پر مبارکباد! آپ نے سیکھا ہے:

✅ این ویڈیا آئزک سم سیٹ اپ اور نیویگیشن
✅ یو ایس ڈی سین کریئیشن اور روبوٹ امپورٹ
✅ سینسر ڈیٹا کے لیے آر او ایس 2 انٹیگریشن
✅ ریپلیکیٹر کے ساتھ سنتھیٹک ڈیٹا جنریشن
✅ ڈومین رینڈمائزیشن حکمت عملی
✅ آئزک جم کے ساتھ بہت زیادہ پیرالل آر ایل
✅ سم ٹو ریئل ٹرانسفر تکنیکیں
✅ سم سے ڈیپلائمنٹ تک مکمل ایم ایل پائپ لائن

## اگلے قدم

**آگے کیا ہے:**
- باب 3 کا اسیسمنٹ پروجیکٹ مکمل کریں
- ضرورت کے مطابق آئزک سم تصورات کا جائزہ لیں
- باب 4 کے لیے تیاری کریں: [ویژن-لینگویج-ایکشن ماڈلز اور کیپ سٹون](../04-vla/index.md)

باب 4 ہر چیز کو اکٹھا کرے گا: ملٹی موڈل اے آئی، اینڈ ٹو اینڈ روبوٹ سسٹمز، اور آپ کا حتمی کیپ سٹون پروجیکٹ!

---

## 📝 ہفتہ وار کوئز

اس ہفتے کے مواد کی اپنی سمجھ کو جانچیں! کوئز ملٹیپل چوائس ہے، خودکار طور پر اسکور ہوتا ہے، اور آپ کے پاس 2 کوششیں ہیں۔

**[ہفتہ 10 کوئز لیں →](/quiz?week=10)**
