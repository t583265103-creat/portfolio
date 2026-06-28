# TrackObject — הערות פרויקט

## מה הפרויקט עושה

מערכת ניווט אוטונומי לכיסא גלגלים המופעלת בסביבת **Webots**. הכיסא מנווט מנקודת GPS לנקודת יעד תוך:
- מיפוי סביבתי בזמן אמת (LiDAR + OctoMap)
- זיהוי ומעקב עצמים (הולכי רגל, רכבים, רמזורים)
- מיזוג חיישנים (GPS, IMU, Encoder, מצלמה, Radar)
- הימנעות ממכשולים (D* Lite + A*)

---

## ארכיטקטורה

### מודולים ראשיים

| תיקייה | תפקיד |
|---|---|
| `WebotsBridge/` | ממשק עם Webots — קריאת חיישנים, שליחת פקודות מנוע |
| `Thread/` | ניהול threads, Singletons, תורי עדיפויות |
| `Lidar/` | עיבוד ענן נקודות |
| `Camera/` | זיהוי עצמים YOLOv8 |
| `Radar/` | זיהוי מטרות + מהירות |
| `Gps/` | פענוח NMEA |
| `Imu/` | תאוצה, ג'ירוסקופ |
| `Encoder/` | אודומטריה מגלגלים |
| `MapLive/` | רשת תאים 2D + OctoMap + D*Lite + A* |
| `SensorFusion/` | Hungarian Algorithm + TrackedObjects |
| `KalmanFilter/` | EKF 8 ממדי |

### Thread Architecture (12 threads)

```
Main Thread (100Hz)
  └─ wb_robot_step → webotsProducer.update() → SensorProducerManager rings

StateThread (1 thread)
  └─ EKF predict+update loop (popTask blocking)

SensorConsumer (6 threads)
  ├─ ConsumerImu     → ring → ImuProcessing    → EKF task
  ├─ ConsumerEncoder → ring → EncoderProcessing → EKF task
  ├─ ConsumerLidar   → ring → LidarProcessing   → FramePointers
  ├─ ConsumerCamera  → ring → YOLOv8            → FramePointers
  ├─ ConsumerRadar   → ring → RadarProcessing   → FramePointers
  └─ ConsumerGps     → ring → GpsProcessing     → FramePointers

ThreadGrafFector (5 threads)
  ├─ ConsumerProcessingFrame → SensorFusion + ICP + NavigationManager
  ├─ Optimization            → FactorGraph optimize → EKF task
  ├─ createNode              → keyframe creation
  ├─ removeNodeAndFector     → sliding window cleanup
  └─ SecurityWall            → בדיקת בטיחות + רמזורים
```

### Singletons (נוצרים לפני הרצת threads)
- `StatePriorityQueue` — EKF 8D + תור עדיפויות לעדכונים
- `SensorFrameManager` — מפה: timestamp → FramePointers
- `SensorProducerManager` — 6 ring buffers לחיישנים
- `FactorGraphManager` — Factor Graph SLAM
- `NavigationManager` — MapData + MapLocal + OctoMap + D*Lite + A*
- `WebotsDrive` — פקודות מנועים

---

## זרימת נתונים

```
Webots → SensorProducerWebots → SensorRing buffers
                                        ↓
                               SensorConsumer threads
                                    ↙        ↘
                           EKF tasks        FramePointers
                               ↓                 ↓
                          StateThread    ThreadGrafFector
                               ↓                 ↓
                           EKF state      SensorFusion + SLAM
                                    ↘    ↙
                               NavigationManager
                                        ↓
                                   WebotsDrive
```

---

## בעיות שנמצאו ותוקנו

### 1. Data Race ב-FactorGraphManager [קריטי — מקור הקראש]

**בעיה:** שלושה threads (`Optimization`, `createNode`, `removeNodeAndFector`) ניגשים לאותם מבני נתונים (`poses`, `factors`, `time_to_id`, `current_node_id`) בו-זמנית ללא mutex.

**תיקון:** הוספת `std::recursive_mutex m_mutex` ונעילה בכל הפונקציות הציבוריות של `FactorGraphManager`.

**קבצים:** `Thread/FactorGraphManager.hpp`, `Thread/FactorGraphManager.cpp`

---

### 2. Ghost Node ב-getLastNode() [קריטי]

**בעיה:** `getLastNode()` משתמשת ב-`poses[current_node_id]` (operator[]) שמכניסה `NodeFactor` ריק (x.size()=0) למפה כש-`poses` ריקה. `optimizeGraph()` אחר כך מנסה לחשב `x += delta_x.segment(...)` על הצומת הריק → crash.

**תיקון:** שינוי ל-`poses.find()` שלא מכניס ערך ריק.

**קבצים:** `Thread/FactorGraphManager.cpp`

---

### 3. SensorWorkerManager sleep ארוך פי 1000 [חמור]

**בעיה:** `std::this_thread::sleep_for(std::chrono::milliseconds(10000))` — ה-consumers רצים פעם בעשר שניות במקום 100Hz.

**תיקון:** `10000` → `10`

**קבצים:** `Thread/SensorWorkerManager.hpp`

---

### 4. Exception handling שבור [חמור]

**בעיה:** כל ה-consumer threads תופסים רק `std::runtime_error`. `cv::Exception` (OpenCV) ו-Eigen exceptions יורשים מ-`std::exception` אך לא מ-`std::runtime_error` — חומקים ל-`std::terminate()` → crash.

**תיקון:** `catch (const std::runtime_error&)` → `catch (const std::exception& e)`

**קבצים:** `Thread/SensorConsumer.cpp`

---

### 5. pqxx include לא שייך [בינוני]

**בעיה:** `SensorFrameManager.hpp` כולל `<pqxx/pqxx>` (PostgreSQL) — ספרייה לא רלוונטית לסימולציה, גורמת ל-pqxx לסמן `auto` כמאקרו מה שגרם ל-`#undef auto` ב-`ThreadGrafFector.cpp`.

**תיקון:** הסרת ה-include ו-`saveFrameToDatabase` לא בשימוש. הסרת `#undef auto`.

**קבצים:** `Thread/SensorFrameManager.hpp`, `Thread/ThreadGrafFector.cpp`

---

## בעיות נוספות שעדיין פתוחות (לא תוקנו)

- `SensorFrameManager::ProcessFrame` לא מתקדם אף פעם — `nextFrame` לא נמצא בשום מקום → `ConsumerProcessingFrame` תמיד מקבל `nullptr` → מיזוג חיישנים לא רץ בכלל.
- `IcpEstimator`, `SensorFusionCamera` וכו' נוצרים כ-local variables בכל קריאה ל-`ConsumerProcessingFrame` — בזבזני.
- GPS: `x_local = 3205.12` — ערך ב-NMEA DDMM.MMMMM, לא decimal degrees. צריך המרה ב-`GpsData::buildZ`.
- `SensorFrameManager::ProcessFrame` — צריך מנגנון `nextFrame` שיקשר frames לפי timestamp.

---

## קונפיגורציה

### Sensor Rates
- IMU / Encoder: 100 Hz (10ms)
- GPS: 10 Hz (100ms)
- LiDAR / Camera / Radar: 2 Hz (500ms)

### EKF State Vector (8D)
`[X, Y, Z, Pitch, Yaw, Vx, V_yaw, Ax]`

### Map
- גודל: 2000×2000 תאים, 0.1 מ'/תא (= 200×200 מטר)
- רזולוציה גלובלית: lat/lon על גרף עם A*
- מקומית: D* Lite על רשת התאים

---

## Build
```
cmake -B build
cmake --build build --config Debug
# פלט: controllers/wheelchair_cpp_controller/wheelchair_cpp_controller.exe
```
