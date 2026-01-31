# Week 13: Capstone Project

## خلاصہ

آخری ہفتے تک پہنچنے پر مبارکباد! Capstone پروجیکٹ ایک مکمل end-to-end روبوٹک سسٹم بنا کر Physical AI میں مہارت کا مظاہرہ کرنے کا آپ کا موقع ہے۔ آپ ROS 2، simulation، synthetic data generation، اور VLA ماڈلز کو ایک کام کرنے والے خودکار روبوٹ میں یکجا کریں گے۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ یہ کر سکیں گے:

- تقاضوں سے لے کر تعیناتی تک مکمل Physical AI سسٹم ڈیزائن کرنا
- متعدد modules (perception، planning، control) کو مربوط طریقے سے یکجا کرنا
- سسٹم کی کارکردگی کا مقداری طور پر جائزہ لینا
- تکنیکی کام کو پیشہ ورانہ انداز میں پیش کرنا
- حدود کی نشاندہی کرنا اور بہتریاں تجویز کرنا

## Capstone Project تفصیل

### پروجیکٹ کا ہدف

**Physical AI تکنیکوں کا استعمال کرتے ہوئے ایک خودکار روبوٹ سسٹم بنائیں جو حقیقی دنیا کا کام انجام دے۔**

### تقاضے (کل 200 پوائنٹس)

#### 1. System Architecture (30 points)

**فراہمی:**
- تمام اجزاء دکھانے والا Architecture diagram
- Data flow diagram (sensors → processing → actuators)
- ROS 2 node graph
- ڈیزائن انتخاب کے لیے تحریری جواز

**اجزاء میں شامل ہونا ضروری ہے:**
- Perception (camera، lidar، یا depth sensor)
- Planning/decision-making (VLA model، روایتی planner، یا RL policy)
- Control (motor commands، trajectory execution)
- Safety/failure handling

#### 2. Robot Modeling & Simulation (30 points)

**فراہمی:**
- روبوٹ کا URDF model (یا پہلے سے بنا ہوا استعمال کریں)
- Gazebo یا Isaac Sim environment
- Sensor integration (کم از کم 2 sensors)
- حقیقت پسند رویے کے لیے Physics tuning

**جائزے کے معیار:**
- ماڈل کی درستگی (حقیقی روبوٹ سے مماثلت اگر تعینات کر رہے ہیں)
- Sensor کی وفاداری
- Physics کی حقیقت پسندی

#### 3. Perception Pipeline (40 points)

**فراہمی:**
- Object detection/segmentation یا scene understanding
- Synthetic data پر تربیت یافتہ یا پہلے سے تربیت یافتہ model
- ریئل ٹائم inference (>5 FPS)
- درستگی کے میٹرکس (mAP، IoU، وغیرہ)

**اختیارات:**
- **Vision-based**: YOLOv8، SegmentAnything، CLIP
- **3D perception**: PointNet++، 3D object detection
- **Multimodal**: RGB-D fusion، sensor fusion

#### 4. Decision-Making/Planning (40 points)

**فراہمی:**
- VLA model یا RL policy یا classical planner
- تربیت یافتہ policy (اگر learning-based ہے)
- Simulation میں کامیابی کی شرح > 70%
- Edge cases کو خوبصورتی سے سنبھالتا ہے

**اختیارات:**
- **VLA**: مظاہرہ ڈیٹا پر تربیت دیں
- **RL**: manipulation/navigation کے لیے PPO/SAC
- **Classical**: path planning کے لیے RRT*، A*

#### 5. System Integration & Testing (30 points)

**فراہمی:**
- مکمل سسٹم شروع کرنے والی ROS 2 launch file
- Simulation میں end-to-end مظاہرہ
- مقداری کارکردگی کے میٹرکس
- ناکامی کے معاملے کا تجزیہ

**ٹیسٹنگ کے تقاضے:**
- کم از کم 50 ٹیسٹ رنز
- کامیابی کی شرح، تکمیل کا وقت، ناکامی کی اقسام رپورٹ کریں
- شماریاتی اہمیت (error bars، confidence intervals)

#### 6. Documentation (20 points)

**فراہمی:**
- سیٹ اپ کی ہدایات کے ساتھ README
- تکنیکی رپورٹ (8-12 صفحات)
- Demo video (5-10 منٹ)
- Code comments اور docstrings

**رپورٹ کے حصے:**
1. Abstract
2. تعارف اور محرک
3. System Design
4. نفاذ کی تفصیلات
5. تجربات اور نتائج
6. بحث اور حدود
7. نتیجہ اور مستقبل کا کام

#### 7. Presentation (10 points)

**فراہمی:**
- 15 منٹ کی پریزنٹیشن
- Slides (10-15 slides)
- Live demo یا demo video

**پریزنٹیشن کی ساخت:**
1. مسئلے کا بیان (2 min)
2. سسٹم کا خلاصہ (3 min)
3. تکنیکی تفصیلات (5 min)
4. نتائج (3 min)
5. سوالات و جوابات (2 min)

## پروجیکٹ کے خیالات

### ابتدائی دوستانہ پروجیکٹس

#### 1. Warehouse Inspection Robot
**کام**: گودام میں navigation، anomalies کی تشخیص (گرے ہوئے boxes، spills)

**اجزاء:**
- Mobile base (differential drive)
- Anomaly detection کے لیے camera
- Navigation کے لیے SLAM
- Object detection (YOLOv8)

**فراہمی:**
- خودکار patrol route
- >85% درستگی کے ساتھ anomaly detection
- Anomaly locations کے ساتھ رپورٹ کی تخلیق

#### 2. Tabletop Sorting Robot
**کام**: اشیاء اٹھائیں اور رنگ/شکل کے لحاظ سے ترتیب دیں

**اجزاء:**
- Robot arm (Franka Panda یا اسی طرح)
- RGB camera
- Pick-and-place VLA policy
- Color classifier

**فراہمی:**
- >80% sorting کی درستگی
- Occlusions کی handling
- Multi-object grasping

#### 3. Autonomous Delivery Robot
**کام**: مقام تک navigation، چیز کی فراہمی

**اجزاء:**
- Container کے ساتھ mobile robot
- رکاوٹوں سے بچاؤ کے لیے Lidar
- Navigation stack (Nav2)
- ترسیل کی تصدیق کے لیے QR code detection

**فراہمی:**
- 3+ waypoints تک navigate کریں
- متحرک رکاوٹوں سے بچیں
- \<5% collision rate

### درمیانی پروجیکٹس

#### 4. Kitchen Assistant Robot
**کام**: آواز کے احکامات کی بنیاد پر shelves سے اشیاء لائیں

**اجزاء:**
- Mobile manipulator
- Object recognition کے لیے vision-language model
- Motion planning (MoveIt2)
- Speech recognition

**فراہمی:**
- 10+ آواز کے احکامات سمجھیں
- 75% وقت درست شے لائیں
- Clutter اور occlusions سنبھالیں

#### 5. Agricultural Inspection Drone
**کام**: فصلوں کا معائنہ، بیماری/کیڑوں کی تشخیص

**اجزاء:**
- Simulation میں quadcopter
- Multispectral camera
- Disease classification model
- خودکار flight planning

**فراہمی:**
- خودکار field coverage
- >80% درستگی کے ساتھ بیماری کی تشخیص
- GPS-tagged anomaly map

#### 6. Collaborative Assembly Robot
**کام**: مصنوعات جمع کرنے کے لیے انسان کے ساتھ تعاون کریں

**اجزاء:**
- Dual-arm robot
- Human pose estimation
- Collision avoidance
- Tool manipulation

**فراہمی:**
- محفوظ انسان-روبوٹ تعاون
- Assembly task کی تکمیل
- انسانی موجودگی پر \<1 سیکنڈ reaction time

### اعلیٰ درجے کے پروجیکٹس

#### 7. Autonomous Drone Racing
**کام**: تیز رفتاری سے رکاوٹوں کے کورس میں navigation

**اجزاء:**
- Vision کے ساتھ quadcopter
- کنٹرول کے لیے VLA model
- RL-trained policy
- Localization کے لیے SLAM

**فراہمی:**
- \<60 سیکنڈ میں کورس مکمل کریں
- صفر ٹکراؤ
- نئے tracks میں عمومیت

#### 8. Dexterous Manipulation
**کام**: ہاتھ میں manipulation (اشیاء کو گھمائیں)

**اجزاء:**
- Multi-fingered gripper
- Tactile sensors
- RL policy (Isaac Gym)
- Sim-to-real transfer

**فراہمی:**
- شے کو 90 ڈگری گھمائیں
- >70% کامیابی کی شرح
- 5+ شے کی اقسام پر کام کرتا ہے

#### 9. Multi-Robot Coordination
**کام**: متعدد روبوٹس کام پر تعاون کریں

**اجزاء:**
- 3+ روبوٹس (یکساں یا مختلف)
- تقسیم شدہ منصوبہ بندی
- Communication protocol
- ہم آہنگی کی حکمت عملی

**فراہمی:**
- ابھرتا ہوا ٹیم کا رویہ
- واحد روبوٹ سے تیز کام کی تکمیل
- روبوٹ کی ناکامیوں کے لیے مضبوط

## نفاذ کا ٹائم لائن

### Week 1: منصوبہ بندی اور سیٹ اپ
- پروجیکٹ کے دائرہ کار کو حتمی شکل دیں
- Architecture diagram بنائیں
- ترقیاتی ماحول سیٹ اپ کریں
- ابتدائی روبوٹ model/simulation بنائیں

### Week 2: بنیادی ترقی
- Perception pipeline نافذ کریں
- Planning/control algorithm تیار کریں
- ROS 2 nodes کو یکجا کریں
- Simulation میں ابتدائی ٹیسٹنگ

### Week 3: بہتری اور ٹیسٹنگ
- Bugs اور edge cases ٹھیک کریں
- مقداری میٹرکس جمع کریں
- کارکردگی کو بہتر بنائیں
- دستاویزات شروع کریں

### Week 4: حتمی شکل
- ٹیسٹنگ مکمل کریں (50+ رنز)
- تکنیکی رپورٹ لکھیں
- Demo video بنائیں
- پریزنٹیشن تیار کریں

## جائزے کا Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Architecture** | 30 | واضح ڈیزائن، اچھی طرح سے جواز، modular |
| **Simulation** | 30 | حقیقت پسند physics، مناسب sensors، مستحکم |
| **Perception** | 40 | درست، ریئل ٹائم، مضبوط |
| **Planning/Control** | 40 | کام حاصل کرتا ہے، ناکامیاں سنبھالتا ہے، موثر |
| **Integration** | 30 | بغیر کسی رکاوٹ کے ROS 2، کوئی اہم bugs نہیں |
| **Documentation** | 20 | واضح، مکمل، پیشہ ورانہ |
| **Presentation** | 10 | دلچسپ، واضح، مہارت کا مظاہرہ |
| **TOTAL** | 200 | |

### Bonus مواقع (زیادہ سے زیادہ +50 points)

- **حقیقی hardware deployment** (+30 points)
- **نیا algorithm/approach** (+20 points)
- **Multi-task صلاحیت** (+15 points)
- **Sim-to-real transfer** (+20 points)
- **Open-source شراکت** (+10 points)
- **غیر معمولی دستاویزات** (+10 points)

## تکنیکی رپورٹ کا سانچہ

### Title Page
- پروجیکٹ کا عنوان
- آپ کا نام
- تاریخ
- کورس کی معلومات

### Abstract (150-200 الفاظ)
خلاصہ: مسئلہ، طریقہ، کلیدی نتائج

### 1. تعارف (1-2 صفحات)
- محرک: یہ مسئلہ کیوں اہم ہے؟
- متعلقہ کام: دوسروں نے کیا کیا ہے؟
- شراکتیں: آپ کے طریقہ کار میں نیا کیا ہے؟

### 2. System Design (2-3 صفحات)
- Architecture کا خلاصہ
- اجزاء کی تفصیلات
- ڈیزائن کا جواز
- غور کردہ متبادل طریقے

### 3. نفاذ (2-3 صفحات)
- روبوٹ model/simulation سیٹ اپ
- Perception pipeline کی تفصیلات
- Planning/control algorithm
- ROS 2 integration
- کلیدی code snippets

### 4. تجربات (2-3 صفحات)
- تجرباتی سیٹ اپ
- جائزے کے میٹرکس
- نتائج (جدولیں، graphs)
- Ablation studies (کون سے اجزاء سب سے زیادہ اہم ہیں؟)
- Baselines سے موازنہ

### 5. بحث (1-2 صفحات)
- نتائج کی تشریح
- حدود اور ناکامی کے معاملات
- سیکھے گئے اسباق
- درستگی کے لیے خطرات

### 6. نتیجہ اور مستقبل کا کام (1 صفحہ)
- شراکتوں کا خلاصہ
- مستقبل کی بہتری کے لیے سفارشات
- وسیع تر اثر

### References
- استعمال شدہ تمام papers، libraries، datasets کا حوالہ دیں

## Demo Video کی ہدایات

**مدت**: 5-10 منٹ

**ساخت:**
1. **تعارف** (30 sec): عنوان، آپ کا نام، پروجیکٹ کا خلاصہ
2. **مسئلے کا بیان** (1 min): آپ کون سا کام حل کر رہے ہیں؟ یہ کیوں اہم ہے؟
3. **System Architecture** (1 min): اعلیٰ سطحی diagram کی واک تھرو
4. **Live Demo** (3-4 min):
   - عمل میں روبوٹ دکھائیں
   - متعدد camera angles
   - کامیابی اور ناکامی کے معاملات
   - ریئل ٹائم میٹرکس overlay
5. **تکنیکی تفصیلات** (2 min): کلیدی algorithms، code snippets دکھائیں
6. **نتائج** (1 min): مقداری میٹرکس، graphs
7. **نتیجہ** (30 sec): خلاصہ، مستقبل کا کام

**تجاویز:**
- 1080p یا اس سے اوپر میں ریکارڈ کریں
- واضح آڈیو (microphone استعمال کریں، laptop mic نہیں)
- کلیدی نکات کے لیے captions شامل کریں
- Background music ٹھیک ہے (کم volume رکھیں)
- Simulation اور حقیقی روبوٹ دونوں دکھائیں (اگر قابل اطلاق ہو)

## مثال کا پروجیکٹ: Autonomous Bin Picking

### سسٹم کا خلاصہ

**کام**: bin سے بے ترتیب رکھے گئے حصوں کو اٹھائیں اور containers میں ترتیب دیں

**اجزاء:**
1. **روبوٹ**: Franka Panda arm with gripper
2. **Perception**: RGB-D camera، object detection (YOLOv8)
3. **Planning**: Grasp pose estimation، motion planning (MoveIt2)
4. **Control**: Joint trajectory execution

### Architecture

```
┌────────────┐     ┌────────────┐     ┌────────────┐
│   Camera   │────▶│   Object   │────▶│   Grasp    │
│   (RGB-D)  │     │  Detection │     │ Estimation │
└────────────┘     └────────────┘     └──────┬─────┘
                                              │
                                              ▼
┌────────────┐     ┌────────────┐     ┌────────────┐
│   Gripper  │◀────│   Motion   │◀────│  Planning  │
│   Control  │     │  Execution │     │  (MoveIt2) │
└────────────┘     └────────────┘     └────────────┘
```

### کلیدی نتائج

- **کامیابی کی شرح**: 87% (100 میں سے 87 picks کامیاب)
- **Cycle Time**: 12.3 ± 2.1 سیکنڈ فی pick
- **ناکامی کی اقسام**:
  - Occlusion (6%)
  - Grasp failure (5%)
  - Planning timeout (2%)

### Code Snippet

```python
class BinPickingNode(Node):
    def __init__(self):
        super().__init__('bin_picking')

        # Perception
        self.detector = YOLOv8Detector(model="yolov8n.pt")

        # Planning
        self.move_group = MoveGroupInterface("panda_arm")

        # Main loop
        self.timer = self.create_timer(0.1, self.pick_callback)

    def pick_callback(self):
        # Get camera image
        image = self.get_camera_image()

        # Detect objects
        detections = self.detector.detect(image)

        if len(detections) > 0:
            # Pick closest object
            target = detections[0]

            # Plan grasp
            grasp_pose = self.compute_grasp_pose(target)

            # Execute
            success = self.move_group.move_to_pose(grasp_pose)

            if success:
                self.get_logger().info("Pick successful!")
```

## جمع کرانے کی Checklist

جمع کرانے سے پہلے، یقینی بنائیں کہ آپ کے پاس:

- [ ] تمام code کے ساتھ GitHub repository
- [ ] سیٹ اپ کی ہدایات کے ساتھ README
- [ ] تکنیکی رپورٹ (PDF)
- [ ] Demo video (YouTube/Vimeo پر اپ لوڈ شدہ)
- [ ] پریزنٹیشن slides (PDF یا PPTX)
- [ ] Dataset/models (اگر بڑا ہو تو cloud link)
- [ ] ROS 2 package بغیر غلطیوں کے compile ہوتا ہے
- [ ] Launch file out-of-box کام کرتی ہے
- [ ] تمام dependencies دستاویز شدہ
- [ ] License file (MIT/Apache تجویز کردہ)

## وسائل

- [ROS 2 Best Practices](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html)
- [Scientific Writing Guide](https://www.nature.com/scitable/topicpage/scientific-writing-13815989/)
- [Demo Video Tips](https://www.youtube.com/watch?v=videoid)
- [LaTeX Report Template](https://www.overleaf.com/latex/templates)

## آخری الفاظ

یہ capstone Physical AI اور Humanoid Robotics میں 13 ہفتوں کی شدید سیکھنے کا نقطہ عروج ہے۔ آپ نے ROS 2 بنیادوں سے لے کر اعلیٰ درجے کی simulation تک جدید ترین VLA ماڈلز تک کا سفر کیا ہے۔

**یاد رکھیں:**
- کمال کی ضرورت نہیں - کلیدی تصورات کے مظاہرے پر توجہ دیں
- اپنی ناکامیوں اور سیکھے گئے اسباق کو دستاویز کریں
- پھنس جائیں تو مدد مانگیں
- کچھ حیرت انگیز بناتے ہوئے مزہ کریں!

**اب آپ لیس ہیں:**
- روبوٹک سسٹمز ڈیزائن اور نافذ کرنے کے لیے
- حقیقی دنیا کی روبوٹکس میں جدید AI/ML استعمال کرنے کے لیے
- Physical AI کے تیزی سے بڑھتے ہوئے شعبے میں شراکت کرنے کے لیے

آپ کے capstone کے لیے نیک خواہشات! ہم یہ دیکھنے کے لیے پرجوش ہیں کہ آپ کیا بناتے ہیں! 🤖🚀

---

## کورس کی تکمیل

### آپ نے کیا سیکھا

13 ہفتوں میں، آپ نے مہارت حاصل کی:

✅ **Chapter 1**: ROS 2 architecture، pub-sub، services، actions
✅ **Chapter 2**: URDF modeling، Gazebo simulation، sensor integration
✅ **Chapter 3**: Isaac Sim، synthetic data، RL، sim-to-real transfer
✅ **Chapter 4**: VLA models، multimodal AI، end-to-end systems

### تکمیل کا سرٹیفکیٹ

کامیاب capstone جمع کرانے پر، آپ حاصل کریں گے:
- تکمیل کا سرٹیفکیٹ
- Portfolio-ready پروجیکٹ
- صنعت سے متعلق مہارتیں
- روبوٹکس/AI میں تحقیق یا کیریئر کے لیے بنیاد

### کورس کے بعد اگلے قدم

**سیکھنا جاری رکھیں:**
- تازہ ترین papers پڑھیں (arXiv: cs.RO، cs.AI)
- روبوٹکس communities میں شامل ہوں (ROS Discourse، Discord)
- Open-source پروجیکٹس میں شراکت کریں
- کانفرنسوں میں شرکت کریں (ICRA، CoRL، IROS)

**کیریئر کے راستے:**
- Robotics Engineer
- AI/ML Engineer (Robotics)
- Research Scientist (Physical AI)
- Autonomous Systems Developer
- Robotics Startup Founder

**منسلک رہیں:**
- GitHub: اپنے پروجیکٹس شیئر کریں
- LinkedIn: ساتھیوں کے ساتھ نیٹ ورک بنائیں
- Twitter/X: روبوٹکس محققین کو فالو کریں
- YouTube: اپنی تعمیرات کو دستاویز کریں

**Physical AI اور Humanoid Robotics میں یہ سفر کرنے کا شکریہ!**

روبوٹکس کا مستقبل آپ کے ہاتھوں میں ہے۔ جائیں اور کچھ ناقابل یقین بنائیں! 🌟
