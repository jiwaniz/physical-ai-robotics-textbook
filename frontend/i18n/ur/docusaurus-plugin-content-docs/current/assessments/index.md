# کورس کی تشخیصات

## خلاصہ

یہ کورس physical AI اور humanoid robotics کی آپ کی سمجھ کا جائزہ لینے کے لیے متعدد تشخیصی طریقے استعمال کرتا ہے۔ تمام تشخیصات نظریاتی علم اور عملی نفاذ کی مہارتوں دونوں کو جانچنے کے لیے ڈیزائن کی گئی ہیں۔

## تشخیص کی ساخت

| Assessment Type | Weight | Due Dates | تفصیل |
|----------------|--------|-----------|-------------|
| **ہفتہ وار Quizzes** | 20% | ہر ہفتے کے اختتام پر | ہفتہ وار مواد کا احاطہ کرنے والے مختصر quizzes |
| **ROS 2 Project** | 15% | Week 5 | ایک multi-node ROS 2 application بنائیں |
| **Gazebo Simulation** | 15% | Week 7 | ایک custom robot بنائیں اور simulate کریں |
| **Isaac Pipeline** | 20% | Week 10 | مکمل Isaac Sim workflow نافذ کریں |
| **Capstone Project** | 30% | Week 13 | End-to-end VLA robotic system |

## تشخیص کی تفصیلات

### 1. ہفتہ وار Quizzes (20%)

**فارمیٹ**: Multiple choice (4 اختیارات، خودکار scored)
**مدت**: ہر quiz کے لیے 15-20 منٹ
**Attempts**: 2 attempts کی اجازت (سب سے زیادہ score شمار ہوتا ہے)
**پاس کرنے کا Score**: 60%

ہفتہ وار quizzes آپ کی سمجھ کو جانچتے ہیں:
- تصوراتی علم (40%)
- Code کی سمجھ (30%)
- Troubleshooting منظرنامے (30%)

#### Quiz لیں

| Week | موضوع | Quiz Link |
|------|-------|-----------|
| 1 | Physical AI بنیادیں | [Quiz شروع کریں](/quiz?week=1) |
| 2 | Dev Environment اور Tools | [Quiz شروع کریں](/quiz?week=2) |
| 3 | ROS 2 بنیادی تصورات | [Quiz شروع کریں](/quiz?week=3) |
| 4 | ROS 2 Communication Patterns | [Quiz شروع کریں](/quiz?week=4) |
| 5 | Services اور Actions | [Quiz شروع کریں](/quiz?week=5) |
| 6 | Gazebo اور URDF بنیادیں | [Quiz شروع کریں](/quiz?week=6) |
| 7 | Sensors اور Perception | [Quiz شروع کریں](/quiz?week=7) |
| 8 | NVIDIA Isaac Sim | [Quiz شروع کریں](/quiz?week=8) |
| 9 | AI کے لیے Synthetic Data | [Quiz شروع کریں](/quiz?week=9) |
| 10 | Sim-to-Real Transfer | [Quiz شروع کریں](/quiz?week=10) |
| 11 | Vision-Language-Action Models | [Quiz شروع کریں](/quiz?week=11) |
| 12 | اعلیٰ درجے کا VLA اور Deployment | [Quiz شروع کریں](/quiz?week=12) |

### 2. ROS 2 Project (15%)

**آخری تاریخ**: Week 5 کے اختتام پر
**جمع کرانا**: GitHub repository + demo video

**تقاضے**:
- کم از کم 3 nodes publishers/subscribers کے ساتھ نافذ کریں
- کم از کم 1 service اور 1 action استعمال کریں
- Launch files اور parameter configuration شامل کریں
- کلیدی اجزاء کے لیے unit tests لکھیں
- سیٹ اپ اور استعمال کی دستاویز کریں

**Rubric**:
- کارکردگی (40%)
- Code کا معیار (25%)
- دستاویزات (20%)
- ٹیسٹنگ (15%)

### 3. Gazebo Simulation Project (15%)

**آخری تاریخ**: Week 7 کے اختتام پر
**جمع کرانا**: URDF files + simulation world + demo video

**تقاضے**:
- کم از کم 4 DOF کے ساتھ custom robot ڈیزائن کریں
- حقیقت پسند physics properties بنائیں
- Sensor integration نافذ کریں (camera، lidar، یا IMU)
- Simulated environment میں کام انجام دیتا ہوا روبوٹ دکھائیں

**Rubric**:
- روبوٹ ڈیزائن (30%)
- Physics کی درستگی (25%)
- Sensor integration (25%)
- کام کی تکمیل (20%)

### 4. Isaac Pipeline Project (20%)

**آخری تاریخ**: Week 10 کے اختتام پر
**جمع کرانا**: Isaac Sim scene + Python scripts + رپورٹ

**تقاضے**:
- مکمل Isaac Sim environment بنائیں
- Synthetic data generation pipeline نافذ کریں
- ایک perception یا control model کو train/fine-tune کریں
- Sim-to-real transfer تحفظات کا مظاہرہ کریں

**Rubric**:
- Environment کا معیار (25%)
- Pipeline کی مکملیت (30%)
- Model کی کارکردگی (30%)
- دستاویزات (15%)

### 5. Capstone Project (30%)

**آخری تاریخ**: Week 13 کے اختتام پر
**جمع کرانا**: مکمل codebase + demo video + تکنیکی رپورٹ + presentation

**تقاضے**:
- VLA model کو robotic system کے ساتھ یکجا کریں
- End-to-end pipeline نافذ کریں (perception → reasoning → action)
- Simulator یا حقیقی hardware پر تعینات کریں
- مضبوط error handling کا مظاہرہ کریں
- جامع دستاویزات فراہم کریں

**Rubric**:
- تکنیکی نفاذ (35%)
- System integration (25%)
- Innovation/creativity (15%)
- دستاویزات (15%)
- Presentation (10%)

## گریڈنگ سکیل

| Grade | فیصد | تفصیل |
|-------|-----------|-------------|
| A+ | 95-100% | غیر معمولی کام |
| A | 90-94% | شاندار سمجھ |
| A- | 85-89% | مضبوط کارکردگی |
| B+ | 80-84% | تصورات کی اچھی گرفت |
| B | 75-79% | تسلی بخش |
| B- | 70-74% | کافی سمجھ |
| C+ | 65-69% | بنیادی قابلیت |
| C | 60-64% | پاس |
| F | \<60% | ناکام |

## جمع کرانے کی ہدایات

### Code جمع کرانا

تمام code میں ہونا ضروری ہے:
- زبان کے مخصوص style guides کی پیروی کریں (Python کے لیے PEP 8، C++ کے لیے Google C++)
- سیٹ اپ کی ہدایات کے ساتھ README شامل کریں
- خودکار linting checks پاس کریں
- جہاں قابل اطلاق ہو unit tests شامل کریں

### Video مظاہرے

تمام demo videos میں ہونا ضروری ہے:
- 3-5 منٹ کی لمبائی میں ہوں
- شروع سے آخر تک مکمل workflow دکھائیں
- آپ کے طریقہ کار کی وضاحت کرتے ہوئے audio commentary شامل کریں
- Error handling اور edge cases کا مظاہرہ کریں

### تحریری رپورٹیں

تمام رپورٹوں میں ہونا ضروری ہے:
- فراہم کردہ LaTeX یا Markdown templates استعمال کریں
- بیرونی وسائل کے لیے مناسب citations شامل کریں
- ڈیزائن فیصلوں اور trade-offs کی وضاحت کریں
- مقداری کارکردگی کے میٹرکس فراہم کریں

## دیر سے جمع کرانے کی پالیسی

- **0-24 گھنٹے دیر سے**: 10% جرمانہ
- **24-48 گھنٹے دیر سے**: 25% جرمانہ
- **48-72 گھنٹے دیر سے**: 50% جرمانہ
- **>72 گھنٹے دیر سے**: قبول نہیں (خاص حالات کے لیے instructor سے رابطہ کریں)

## تعلیمی دیانتداری

- آپ ساتھیوں کے ساتھ تصورات پر بحث کر سکتے ہیں، لیکن تمام code آپ کا اپنا ہونا چاہیے
- استعمال شدہ تمام بیرونی وسائل اور libraries کا حوالہ دیں
- دوسرے طلباء کے ساتھ حل یا مکمل code شیئر نہ کریں
- سرقہ کی صورت میں صفر کریڈٹ اور ممکنہ کورس میں ناکامی ہوگی

## مدد حاصل کرنا

اگر آپ کسی تشخیص میں مشکلات کا سامنا کر رہے ہیں:

1. **RAG chatbot استعمال کریں** textbook میں فوری سوالات کے لیے
2. **Office hours میں شرکت کریں** ذاتی رہنمائی کے لیے
3. **Discussion forums میں پوسٹ کریں** ساتھیوں کی مدد کے لیے
4. **Extension کی درخواست کریں** اگر جائز مشکلات کا سامنا ہو (آخری تاریخ سے پہلے درخواست کرنی ضروری ہے)

## Regrade کی درخواستیں

اگر آپ کو یقین ہے کہ آپ کے کام کو غلط طریقے سے گریڈ کیا گیا:

1. گریڈ ملنے کے بعد 24 گھنٹے انتظار کریں
2. Rubric اور feedback کو احتیاط سے دیکھیں
3. 7 دن کے اندر کورس پلیٹ فارم کے ذریعے regrade کی درخواست جمع کرائیں
4. ہر متنازعہ نکتے کے لیے مخصوص جواز شامل کریں

---

**شروع کرنے کے لیے تیار ہیں؟** [Course Schedule](../intro.md) کا جائزہ لیں اور [Chapter 1: ROS 2 Fundamentals](../01-ros2/index.md) سے شروع کریں!
