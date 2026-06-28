
Claude finished the response

#include "NavigationManager.hpp" #include "MapLive/OctomapAdapter.hpp" // ← חייב להיות לפני השימוש ב-m_octomap #include "WheelchairBinding.hpp" void NavigationManager::updateFromLidar(const std::shared_ptr<FramePointers>& frame, float robot_x, float robot

pasted


#pragma once #include <memory> #include <mutex> #include <atomic> #include <vector> #include <unordered_map> #include <optional> #include <chrono> #include "MapData.hpp" #include "MapLocal.hpp" #include "OctomapAdapter.hpp" #include "SensorFrame.hpp" #include "TrackedObject/TrackedOb

pasted


[main] Building folder: c:/Users/User/Desktop/TrackObject/build [build] Starting build [proc] Executing command: "C:\Program Files\CMake\bin\cmake.EXE" --build c:/Users/User/Desktop/TrackObject/build --config Debug --target ALL_BUILD -j 12 -- [build] MSBuild version 17.12.12+1cce77968 for .NET F

pasted


#include "MapLocal.hpp" #include "OctomapAdapter.hpp" #include "TrackedObject/TrackedObject.hpp" #include "GraphMap.hpp" #include <cmath> #include <algorithm> #include <unordered_set> #include <memory> #include <mutex> #include <shared_mutex> static constexpr float DEFAULT_CORRIDOR_WIDTH_M

pasted


#include "DLite.hpp" // ========================================== // Key operators // ========================================== bool DLite::Key::operator>(const Key& o) const { return k1 > o.k1 || (k1 == o.k1 && k2 > o.k2); } bool DLite::Key::operator<(const Key& o) const { ret

pasted

העלתי לך קוד ושגיאות
 ואת הקוד שהארכיקטורה מתבססת עליו וגם את הארכיטקטורה שאני רוצה שיהיה בקוד 
אני רוצה שתסמן לי איפה עשית תיקון או שינוי
אני צריכה להשלים את הפונקציה RunDLite() בפרויקט ניווט לכיסא גלגלים אוטונומי.
הלוגיקה הרצויה:
הפונקציה מקבלת Frame עדכני ורשימת אובייקטים/מכשולים, קוראת את מיקום הכיסא מה־EKF, ממירה אותו לתא במפה המקומית, ומעדכנת את מפת העלויות הדינמית סביב הכיסא. לאחר מכן היא בוחרת יעד מקומי: אם הצומת הגלובלי הבא נמצא בתוך חלון המפה המקומית — משתמשים בו כיעד; אם לא — מחשבים waypoint ביניים על הכיוון לצומת הגלובלי, במרחק קצר ובתוך גבולות המפה.
לאחר בחירת היעד, הפונקציה בונה רשימת תאים שהשתנו מאז הפריים הקודם, כולל מכשולים חדשים, מכשולים שהוסרו, אובייקטים דינמיים, אזורי אינפלציה ושינויים בעלות. את השינויים האלה מעבירים ל־DLite.updateAndReplan() יחד עם תא המיקום הנוכחי של הרובוט.
אם D* Lite מחזיר מסלול תקין, הפונקציה שומרת את המסלול המקומי, בוחרת את התא הבא לנסיעה, ומתרגמת אותו לפקודת תנועה/יעד מקומי. אם אין מסלול, יש לעצור את הכיסא ולהדליק flag של חסימה דינמית.
בנוסף, אם הכיסא הגיע קרוב מספיק ליעד המקומי או לצומת הגלובלי הבא, הפונקציה מקדמת את האינדקס במסלול הגלובלי לצומת הבא. אם הגיע ליעד הסופי — עוצרת את הניווט.
חשוב:
- לא לאתחל את D* Lite בכל פריים.
- לא לבנות מפה מחדש אם רק חלק מהתאים השתנו.
- לעדכן רק תאים שהשתנו.
- לשמור start קודם כדי לעדכן km.
- לוודא שהיעד המקומי תמיד נמצא בתוך חלון המפה.
- לעצור אם יש דגלי בטיחות: הולך רגל, רמזור אדום, חסימת מעבר חציה או מכשול קרוב.

// DLite.hpp
#pragma once

#include <array>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>
#include <utility>
#include "MapLocal.hpp"


static constexpr float INF = std::numeric_limits<float>::infinity();

class DLite {
public:
    struct Key {
        float k1, k2;
        bool operator>(const Key& o) const;
        bool operator<(const Key& o) const;
    };

    struct Cell {

        float g   = INF;
        float rhs = INF;
    };

    using PQItem = std::pair<Key, int>;
    DLite() = default;
    DLite(int w, int h, float res);
    float getStartG() const { return cells[start_].g; }
    void initialize(int start_col, int start_row,
                    int goal_col,  int goal_row,
                    const std::array<uint8_t, GRID_W * GRID_H>& fused_map);

    void updateAndReplan(
        const std::vector<CellUpdate>& updates,
        int robot_col, int robot_row);

    std::vector<std::pair<int,int>> getPath() const;
    void setGoal(float new_col,float new_row);

private:
    int   W_   = GRID_W;
    int   H_   = GRID_H;
    float RES_ = 1.0f;

    int   start_ = 0;
    int   goal_  = 0;
    float km_    = 0.0f;

    std::vector<uint8_t> cost_map_;
    std::vector<Cell> cells;

    std::priority_queue
        PQItem,
        std::vector<PQItem>,
        std::greater<PQItem>
    > pq_;

    // std::unordered_map<int, Key> in_queue_;

    int                flat(int col, int row) const;
    std::pair<int,int> toColRow(int idx) const;
    float              h(int a, int b) const;
    float              moveCost(int from, int to) const;
    Key                calcKey(int s) const;
    void               computeShortestPath();
    void               updateCell(int u);
    std::vector<int>   neighbors(int idx) const;
};
#pragma once
#include <vector>          // ← היה חסר
#include <array>
#include <cstdint>
#include <cmath>
#include <shared_mutex>
#include <memory>
#include <octomap/OcTree.h>
#include "ConfigNavigation.hpp"
struct CellUpdate {
    int col;
    int row;
    uint8_t new_cost;
};
struct MapData {
    std::unique_ptr<octomap::OcTree> octree;

    // שכבת ליידר
    std::array<uint8_t, GRID_W * GRID_H> lidarLayer      {};
    std::array<float,   GRID_W * GRID_H> lidarConfidence {};

    // שכבת אובייקטים
    std::array<uint8_t, GRID_W * GRID_H> objectLayer      {};
    std::array<float,   GRID_W * GRID_H> objectConfidence {};
    std::array<uint8_t, GRID_W * GRID_H> ownerCount       {};

    std::array<uint8_t, GRID_W * GRID_H> inflationRadiusMap {};
    std::array<uint8_t, GRID_W * GRID_H> costMapFused {};
    std::array<uint8_t, GRID_W * GRID_H> routeLimitLayer {};

    std::vector<CellUpdate> cells;
    float originX = 0.f;
    float originY = 0.f;
    double origin_lat = 0.0;
    double origin_lon = 0.0;

    mutable std::shared_mutex rwMutex;

    MapData() : octree(std::make_unique<octomap::OcTree>(RESOLUTION)) {
        octree->setOccupancyThres(0.5);
        octree->setProbHit(0.7);
        octree->setProbMiss(0.4);
        octree->setClampingThresMin(0.12);
        octree->setClampingThresMax(0.97);

        const uint8_t base_r =
            static_cast<uint8_t>(std::ceil(INFLATION_R / RESOLUTION));
        lidarLayer.fill(COST_UNKNOWN);
        lidarConfidence.fill(0.0f);
        objectLayer.fill(COST_UNKNOWN);
        objectConfidence.fill(0.0f);
        ownerCount.fill(0);
        inflationRadiusMap.fill(base_r);
        costMapFused.fill(COST_UNKNOWN);
        routeLimitLayer.fill(COST_LETHAL);   // ← ברירת מחדל: הכל חסום

    }

    MapData(const MapData&)            = delete;
    MapData& operator=(const MapData&) = delete;
   inline bool inBounds(int row, int col) const {
        return row >= 0 && row < GRID_H && col >= 0 && col < GRID_W;
    }

    inline int flatIdx(int row, int col) const {
        return row * GRID_W + col;
    }

    inline bool worldToGrid(float wx, float wy, int& col, int& row) const {
        col = static_cast<int>((wx - originX) / RESOLUTION);
        row = static_cast<int>((wy - originY) / RESOLUTION);
        return inBounds(row, col);
    }

    inline void gridToWorld(int col, int row, float& wx, float& wy) const {
        wx = originX + (col + 0.5f) * RESOLUTION;
        wy = originY + (row + 0.5f) * RESOLUTION;
    }

    // ── שאילתות ניווט ───────────────────────────────────────────────
    inline bool isCellNavigable(int col, int row) const {
        if (!inBounds(row, col)) return false;
        int idx = flatIdx(row, col);
        if (routeLimitLayer[idx] == COST_LETHAL) return false;
        return costMapFused[idx] < COST_INFLATED;
    }

    inline const std::array<uint8_t, GRID_W * GRID_H>& getFusedMap() const {
        return costMapFused;
    }

    inline std::vector<CellUpdate> getCells() {
        return cells;
    }
};
#pragma once

#include <vector>
#include <cstdint>
#include <cmath>

// ─────────────────────────────────────────────
// טיפוסי נתונים בסיסיים למפת כיסא גלגלים
// ─────────────────────────────────────────────

enum SurfaceType : uint8_t {
    ASPHALT        = 0,
    CONCRETE       = 1,
    PAVING_STONES  = 2,
    COBBLESTONE    = 3,
    GRAVEL         = 4,
    GRASS          = 5,
    SAND           = 6,
    UNKNOWN        = 7
};

struct Edge {
    uint64_t    target;
    double      effort_cost;
    double      length;
    double      width;
    double      incline;
    SurfaceType surface;
    double      safe_speed;
};

struct Node {
    // נתוני מיקום
    double lat = 0.0;
    double lon = 0.0;

    // נתוני נגישות
    double kerb_height      = 0.0;
    bool   is_crossing      = false;
    bool   has_traffic_light= false;

    // רשימת סמיכויות
    std::vector<Edge> edges;

    // ── משתני עזר לאלגוריתמי ניווט (לא נשמרים ב-Binary) ──
    double  g_value = 1e9;     // עלות מצטברת מהמוצא
    double  h_value = 0.0;     // הערכת מרחק לייעד (heuristic)
    Node*   parent  = nullptr; // מצביע להורה במסלול
    bool    visited = false;   // סמן לבדיקת ביקור

    double f_value() const { return g_value + h_value; }
};
Show more
23:10
אנתח את השגיאות ואתקן את הקוד. השגיאות העיקריות:

MapLocal::buildFromOctomap — מוגדר ללא פרמטרים אבל נקרא עם m_octomap
MapLocal::worldToGrid, inBounds, isCellNavigable — פרטיים ב-MapLocal אבל NavigationManager צריך אותם
ObjectEntry::updated/removed — שמות שדות שגויים (צריך TrackerFrame)
DLite::INF — INF מוגדר גלובלי, לא כחבר של DLite
NODE_REACH_THRESHOLD — לא מוגדר ב-header
in_queue_ — מוגדר בגוף אבל מוגש בהדר (מוגרש)
GraphResult, buildGraphFromPython — חסרים includes
runAStar — לא מוגדר בהדר
הנה הקבצים המתוקנים:

NavigationManager.hpp — שינויים מסומנים ב-// ← תיקון:

cpp
#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <optional>
#include <chrono>

#include "MapData.hpp"
#include "MapLocal.hpp"
#include "OctomapAdapter.hpp"
#include "SensorFrame.hpp"
#include "TrackedObject/TrackedObject.hpp"
#include "DLite.hpp"
#include "AStar.hpp"
#include "GraphMap.hpp"
#include "Location.hpp"
#include "GraphBuilder.hpp"   // ← תיקון: חסר include עבור buildGraphFromPython / GraphResult

// ← תיקון: NODE_REACH_THRESHOLD לא היה מוגדר בשום מקום
static constexpr float NODE_REACH_THRESHOLD = 3.0f;

class NavigationManager {
public:
    static NavigationManager& getInstance() {
        static NavigationManager instance;
        return instance;
    }

    std::shared_ptr<MapData> getMapData() const {
        return m_mapData;
    }

    // ---------- Global navigation ----------
    void createMapGlobal(Location myplace, Location mygoal);
    void tryAdvanceGlobalNode(const Eigen::VectorXd& state);
    const Node& getNodeById(uint64_t id) const;
    std::vector<uint64_t> getPathGlobal() const;

    std::vector<uint64_t> getPateGlobal() const {
        return getPathGlobal();
    }

    // ---------- Local map ----------
    void updateFromLidar(
        const std::shared_ptr<FramePointers>& frame,
        float robot_x,
        float robot_y
    );

    // ← תיקון: החתימה הישנה קיבלה רק ObjectEntry — עודכנה להתאים להדר
    void updateFromObjects(
        const ObjectEntry& batch,
        std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects
    );

    void finalize();

    // ---------- D* Lite ----------
    void runDLite(
        const std::shared_ptr<FramePointers>& frame,
        const std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects,
        const Eigen::VectorXd& state
    );

    // ---------- Stop / Safety flags ----------
    void setDynamicBlocked(bool blocked);
    void setTrafficBlocked(bool blocked);
    void setCrossingBlocked(bool blocked);
    bool shouldStop() const;

    bool stoppedByDynamic()  const { return m_dynamicBlocked.load();  }
    bool stoppedByTraffic()  const { return m_trafficBlocked.load();  }
    bool stoppedByCrossing() const { return m_crossingBlocked.load(); }

    // ---------- Local path ----------
    void updatePath(std::vector<std::pair<int,int>> newPath);
    std::optional<std::pair<int,int>> getNextStep();
    void popNextStep();
    std::vector<std::pair<int,int>> getPathLiveTime() const;

    // ---------- Current global node / edge ----------
    Node* getCurrentNode() const;
    Edge* getCurrentEdge() const;

    Location getLocation() const { return myGoal; }

private:
    NavigationManager()
        : m_mapData(std::make_shared<MapData>())
        , m_mapLocal(m_mapData)          // ← תיקון: MapLocal בנאי דורש shared_ptr<MapData>
    {}

    NavigationManager(const NavigationManager&)            = delete;
    NavigationManager& operator=(const NavigationManager&) = delete;

    void updateCurrentEdge();

    bool chooseLocalGoal(
        float robot_x, float robot_y,
        int robot_col, int robot_row,
        int& goal_col, int& goal_row
    );

    bool chooseLookaheadGoal(
        float robot_x, float robot_y,
        int robot_col, int robot_row,
        int& goal_col, int& goal_row
    );

    bool goalChanged(int goal_col, int goal_row) const;

    bool findAlternativeLocalGoal(
        int goal_col, int goal_row,
        int& alt_col, int& alt_row
    );

private:
    std::shared_ptr<MapData> m_mapData;
    OctomapAdapter           m_octomap;
    MapLocal                 m_mapLocal;
    DLite                    m_dlite;
    AStar                    m_astar;

    bool m_dliteInitialized = false;
    int  m_lastGoalCol      = -1;
    int  m_lastGoalRow      = -1;

    std::atomic_bool m_dynamicBlocked  {false};
    std::atomic_bool m_trafficBlocked  {false};
    std::atomic_bool m_crossingBlocked {false};

    std::chrono::steady_clock::time_point m_lastDynamicDangerTime =
        std::chrono::steady_clock::time_point::min();

    std::vector<std::pair<int,int>> m_currentPath;
    mutable std::mutex              m_pathMutex;

    std::unordered_map<uint64_t, Node> myMap;
    std::vector<uint64_t>              m_globalPath;
    std::size_t                        m_globalPathIndex = 0;

    Node* m_currentNode = nullptr;
    Edge* m_currentEdge = nullptr;

    Location myGoal;
};
NavigationManager.cpp — שינויים מסומנים:

cpp
#include "NavigationManager.hpp"
#include "MapLive/OctomapAdapter.hpp"
#include "WheelchairBinding.hpp"
#include "GraphBuilder.hpp"   // ← תיקון: buildGraphFromPython + GraphResult

void NavigationManager::updateFromLidar(const std::shared_ptr<FramePointers>& frame,
                                        float robot_x, float robot_y)
{
    m_octomap.updateFromFrame(frame);
    m_mapLocal.shiftOrigin(robot_x, robot_y);
    m_mapLocal.buildFromOctomap();   // ← תיקון: הפונקציה לא מקבלת פרמטר — m_octomap משותף דרך MapData
    m_mapLocal.inflationPass();
}

// ← תיקון: החתימה עודכנה — הוספנו trackedObjects כדי להתאים להדר ול-MapLocal::updateObjects
void NavigationManager::updateFromObjects(
    const ObjectEntry& batch,
    std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects)
{
    // ← תיקון: ObjectEntry הומר ל-TrackerFrame שהוא הטיפוס שMapLocal מצפה לו
    TrackerFrame frame;
    frame.updated = batch.updated;
    frame.removed = batch.removed;
    m_mapLocal.updateObjects(frame, trackedObjects);
}

// ← הוסרה updateCurbs — לא מוגדרת בהדר; הפונקציונליות קיימת ב-MapLocal::buildRouteLimitLayer

void NavigationManager::finalize()
{
    m_mapLocal.fuseMaps();
}

// ← תיקון: runAStar הוסרה מהקוד — לא הוגדרה בהדר; הקריאה ל-A* מתבצעת בתוך createMapGlobal

void NavigationManager::createMapGlobal(Location myplace, Location mygoal)
{
    myGoal = mygoal;

    GraphResult res = buildGraphFromPython(
        myplace.lat, myplace.lon,
        mygoal.lat,  mygoal.lon,
        "wheelchair_config.json"
    );

    myMap = std::move(res.graph);

    m_currentNode = nullptr;
    m_currentEdge = nullptr;

    auto it = myMap.find(res.source_node);
    if (it != myMap.end()) {
        m_currentNode = &it->second;
    } else {
        std::cerr << "[NavigationManager] source node not found\n";
        return;
    }

    std::cout << "[NavigationManager] graph loaded: " << myMap.size() << " nodes\n"
              << "  source: " << res.source_node
              << " | target: " << res.target_node << "\n";

    // ← תיקון: m_astar.run מקבל 3 פרמטרים — myMap, source, target (לא 1 כפי שהיה)
    m_globalPath = m_astar.run(myMap, res.source_node, res.target_node);

    if (m_globalPath.empty()) {
        std::cerr << "[NavigationManager] A* found no path\n";
        return;
    }

    m_globalPathIndex = 0;

    if (m_globalPath.size() > 1) {
        uint64_t next_id = m_globalPath[1];
        for (auto& e : m_currentNode->edges) {
            if (e.target == next_id) {
                m_currentEdge = &e;
                break;
            }
        }
    }

    std::cout << "[NavigationManager] path: " << m_globalPath.size() << " nodes\n";
}

const Node& NavigationManager::getNodeById(uint64_t id) const
{
    auto it = myMap.find(id);
    if (it == myMap.end())
        throw std::out_of_range("[NavigationManager::getNodeById] id not found");
    return it->second;
}

void NavigationManager::setDynamicBlocked(bool blocked)
{
    m_dynamicBlocked.store(blocked);
    if (blocked)
        m_lastDynamicDangerTime = std::chrono::steady_clock::now();
}

void NavigationManager::setTrafficBlocked(bool blocked)  { m_trafficBlocked.store(blocked);  }
void NavigationManager::setCrossingBlocked(bool blocked) { m_crossingBlocked.store(blocked); }

bool NavigationManager::shouldStop() const
{
    constexpr auto DYNAMIC_HOLD_TIME = std::chrono::milliseconds(800);
    const auto now = std::chrono::steady_clock::now();

    bool dynamicHold = false;
    if (m_lastDynamicDangerTime != std::chrono::steady_clock::time_point::min())
        dynamicHold = (now - m_lastDynamicDangerTime) < DYNAMIC_HOLD_TIME;

    return m_dynamicBlocked.load()
        || dynamicHold
        || m_trafficBlocked.load()
        || m_crossingBlocked.load();
}

std::vector<uint64_t> NavigationManager::getPathGlobal() const { return m_globalPath; }

void NavigationManager::updatePath(std::vector<std::pair<int,int>> newPath)
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    m_currentPath = std::move(newPath);
}

std::optional<std::pair<int,int>> NavigationManager::getNextStep()
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    if (m_currentPath.empty()) return std::nullopt;
    return m_currentPath.front();
}

void NavigationManager::popNextStep()
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    if (!m_currentPath.empty())
        m_currentPath.erase(m_currentPath.begin());
}

std::vector<std::pair<int,int>> NavigationManager::getPathLiveTime() const
{
    std::lock_guard<std::mutex> lock(m_pathMutex);
    return m_currentPath;
}

Node* NavigationManager::getCurrentNode() const { return m_currentNode; }
Edge* NavigationManager::getCurrentEdge() const { return m_currentEdge; }

void NavigationManager::tryAdvanceGlobalNode(const Eigen::VectorXd& state)
{
    if (m_globalPath.empty()) return;
    if (m_globalPathIndex + 1 >= m_globalPath.size()) return;

    uint64_t nextNodeId = m_globalPath[m_globalPathIndex + 1];
    auto nextIt = myMap.find(nextNodeId);
    if (nextIt == myMap.end()) return;

    const Node& nextNode = nextIt->second;

    constexpr double METERS_PER_DEG_LAT = 111320.0;
    const double lat_rad = nextNode.lat * (M_PI / 180.0);
    const float node_x = static_cast<float>(nextNode.lon * METERS_PER_DEG_LAT * std::cos(lat_rad));
    const float node_y = static_cast<float>(nextNode.lat * METERS_PER_DEG_LAT);

    const float robot_x = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateX)));
    const float robot_y = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateY)));

    const float dx   = robot_x - node_x;
    const float dy   = robot_y - node_y;
    const float dist = std::sqrt(dx*dx + dy*dy);

    if (dist > NODE_REACH_THRESHOLD) return;

    ++m_globalPathIndex;

    auto currentIt = myMap.find(m_globalPath[m_globalPathIndex]);
    if (currentIt != myMap.end())
        m_currentNode = &currentIt->second;

    updateCurrentEdge();

    std::cout << "[Nav] advanced to node " << m_globalPath[m_globalPathIndex]
              << " (" << m_globalPathIndex << "/" << (m_globalPath.size()-1) << ")\n";
}

void NavigationManager::updateCurrentEdge()
{
    m_currentEdge = nullptr;
    if (m_globalPath.empty()) return;
    if (m_globalPathIndex + 1 >= m_globalPath.size()) return;

    auto currentIt = myMap.find(m_globalPath[m_globalPathIndex]);
    if (currentIt == myMap.end()) return;

    uint64_t nextId = m_globalPath[m_globalPathIndex + 1];
    for (auto& edge : currentIt->second.edges) {
        if (edge.target == nextId) {
            m_currentEdge = &edge;
            return;
        }
    }
}

bool NavigationManager::goalChanged(int goal_col, int goal_row) const
{
    return goal_col != m_lastGoalCol || goal_row != m_lastGoalRow;
}

bool NavigationManager::chooseLookaheadGoal(
    float robot_x, float robot_y,
    int   robot_col, int robot_row,
    int&  goal_col, int& goal_row)
{
    constexpr float LOOKAHEAD_METERS = 5.0f;

    if (!m_currentEdge) return false;
    if (m_globalPath.empty()) return false;
    if (m_globalPathIndex + 1 >= m_globalPath.size()) return false;

    uint64_t nextNodeId = m_globalPath[m_globalPathIndex + 1];
    auto nextIt = myMap.find(nextNodeId);
    if (nextIt == myMap.end()) return false;

    const Node& nextNode = nextIt->second;
    constexpr double METERS_PER_DEG_LAT = 111320.0;
    const double lat_rad = nextNode.lat * (M_PI / 180.0);

    const float next_x = static_cast<float>(nextNode.lon * METERS_PER_DEG_LAT * std::cos(lat_rad));
    const float next_y = static_cast<float>(nextNode.lat * METERS_PER_DEG_LAT);

    float dir_x = next_x - robot_x;
    float dir_y = next_y - robot_y;
    const float len = std::sqrt(dir_x*dir_x + dir_y*dir_y);
    if (len < 0.001f) return false;

    dir_x /= len;
    dir_y /= len;

    const float lx = robot_x + dir_x * LOOKAHEAD_METERS;
    const float ly = robot_y + dir_y * LOOKAHEAD_METERS;

    int col = 0, row = 0;
    // ← תיקון: worldToGrid ו-isCellNavigable הועברו ל-MapData (ציבורי) — גישה דרך m_mapData
    if (!m_mapData->worldToGrid(lx, ly, col, row)) return false;
    if (!m_mapData->isCellNavigable(col, row))     return false;

    goal_col = col;
    goal_row = row;
    return true;
}

bool NavigationManager::findAlternativeLocalGoal(
    int goal_col, int goal_row,
    int& alt_col, int& alt_row)
{
    constexpr int SEARCH_RADIUS = 8;
    float bestDist = std::numeric_limits<float>::max();
    bool found = false;

    for (int dr = -SEARCH_RADIUS; dr <= SEARCH_RADIUS; ++dr) {
        for (int dc = -SEARCH_RADIUS; dc <= SEARCH_RADIUS; ++dc) {
            int col = goal_col + dc;
            int row = goal_row + dr;

            // ← תיקון: גישה דרך m_mapData במקום m_mapLocal (שיטות היו פרטיות)
            if (!m_mapData->inBounds(row, col))        continue;
            if (!m_mapData->isCellNavigable(col, row)) continue;

            float dist = std::sqrt(static_cast<float>(dc*dc + dr*dr));
            if (dist < bestDist) {
                bestDist = dist;
                alt_col  = col;
                alt_row  = row;
                found    = true;
            }
        }
    }
    return found;
}

// ← תיקון: chooseLocalGoal מוסמת אבל לא מומשה — מוסיפים מימוש
bool NavigationManager::chooseLocalGoal(
    float robot_x, float robot_y,
    int   robot_col, int robot_row,
    int&  goal_col, int& goal_row)
{
    // ניסיון ראשון: lookahead לאורך הכיוון לצומת הגלובלי הבא
    if (chooseLookaheadGoal(robot_x, robot_y, robot_col, robot_row, goal_col, goal_row))
        return true;

    // גיבוי: הצומת הגלובלי הבא עצמו, אם נמצא בתוך הגריד
    if (m_globalPathIndex + 1 < m_globalPath.size()) {
        uint64_t nextId = m_globalPath[m_globalPathIndex + 1];
        auto it = myMap.find(nextId);
        if (it != myMap.end()) {
            constexpr double M_PER_DEG = 111320.0;
            const Node& n = it->second;
            const double lat_rad = n.lat * (M_PI / 180.0);
            float nx = static_cast<float>(n.lon * M_PER_DEG * std::cos(lat_rad));
            float ny = static_cast<float>(n.lat * M_PER_DEG);

            int col = 0, row = 0;
            if (m_mapData->worldToGrid(nx, ny, col, row) &&
                m_mapData->isCellNavigable(col, row)) {
                goal_col = col;
                goal_row = row;
                return true;
            }
        }
    }
    return false;
}

void NavigationManager::runDLite(
    const std::shared_ptr<FramePointers>& frame,
    const std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects,
    const Eigen::VectorXd& state)
{
    if (!frame) { updatePath({}); return; }
    if (shouldStop()) { updatePath({}); return; }

    const float robot_x = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateX)));
    const float robot_y = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateY)));

    m_mapLocal.shiftOrigin(robot_x, robot_y);
    m_octomap.updateFromFrame(frame);
    m_mapLocal.buildFromOctomap();   // ← תיקון: ללא פרמטר

    // ← תיקון: ObjectEntry → TrackerFrame; batch.updated/removed → frame fields
    TrackerFrame batch;
    batch.updated = frame->idObjectUpdate;
    batch.removed = frame->idObjectRemove;

    auto trackedObjectsCopy = trackedObjects;
    m_mapLocal.updateObjects(batch, trackedObjectsCopy);
    m_mapLocal.inflationPass();
    m_mapLocal.fuseMaps();

    int robot_col = 0, robot_row = 0;
    // ← תיקון: גישה דרך m_mapData
    if (!m_mapData->worldToGrid(robot_x, robot_y, robot_col, robot_row)) {
        updatePath({});
        m_dliteInitialized = false;
        return;
    }

    tryAdvanceGlobalNode(state);

    int goal_col = 0, goal_row = 0;
    if (!chooseLocalGoal(robot_x, robot_y, robot_col, robot_row, goal_col, goal_row)) {
        updatePath({});
        m_dliteInitialized = false;
        return;
    }

    const bool needReset = !m_dliteInitialized || goalChanged(goal_col, goal_row);

    if (needReset) {
        m_dlite.initialize(robot_col, robot_row, goal_col, goal_row,
                           m_mapData->getFusedMap());
        m_dliteInitialized = true;
        m_lastGoalCol = goal_col;
        m_lastGoalRow = goal_row;
    } else {
        // ← תיקון: use MapLocal's atomic flush to supply changed cells to D* Lite
        m_dlite.updateAndReplan(m_mapLocal.flushCells(), robot_col, robot_row);
    }

    std::vector<std::pair<int,int>> newPath = m_dlite.getPath();

    // ← תיקון: INF הוא קבוע גלובלי — לא DLite::INF
    const bool noPath = newPath.empty() || m_dlite.getStartG() >= INF;

    if (noPath) {
        int alt_col = -1, alt_row = -1;
        if (findAlternativeLocalGoal(goal_col, goal_row, alt_col, alt_row)) {
            m_dlite.initialize(robot_col, robot_row, alt_col, alt_row,
                               m_mapData->getFusedMap());
            m_dliteInitialized = true;
            m_lastGoalCol = alt_col;
            m_lastGoalRow = alt_row;
            newPath = m_dlite.getPath();

            if (newPath.empty() || m_dlite.getStartG() >= INF) {  // ← תיקון: INF גלובלי
                updatePath({});
                return;
            }
            updatePath(std::move(newPath));
            return;
        }
        updatePath({});
        m_dliteInitialized = false;
        return;
    }

    updatePath(std::move(newPath));
}
MapLocal.hpp — צריך לפתוח את השיטות לציבורי:

cpp
// ← תיקון: worldToGrid, inBounds, isCellNavigable היו private — הועברו ל-MapData (ראה MapData.hpp)
// ב-MapLocal עצמה — אם עדיין נדרשות, להוסיף public wrappers:
public:
    bool worldToGrid(float wx, float wy, int& col, int& row) const {
        return m_data->worldToGrid(wx, wy, col, row);   // ← תיקון: delegate ל-MapData
    }
    bool inBounds(int row, int col) const {
        return m_data->inBounds(row, col);
    }
    bool isCellNavigable(int col, int row) const {
        return m_data->isCellNavigable(col, row);
    }
DLite.hpp — תיקון in_queue_:

cpp
// ← תיקון: in_queue_ היה מוגרש (commented out) בהדר אבל שימש בקוד — מחזירים אותו
#include <unordered_map>

// בתוך private:
std::unordered_map<int, Key> in_queue_;   // ← תיקון: שחזור השדה שהוסר בטעות
סיכום כל התיקונים:

#	שגיאה	תיקון
1	buildFromOctomap(m_octomap) — פרמטר לא קיים	הסרת הפרמטר, קריאה ללא ארגומנטים
2	updateFromObjects חתימה לא תואמת	הוספת trackedObjects כפרמטר שני
3	updateCurbs לא בהדר	הוסרה לחלוטין
4	runAStar לא בהדר	הוסרה, הקריאה ל-A* נשארת ב-createMapGlobal
5	DLite::INF — INF הוא גלובלי	שינוי ל->= INF
6	NODE_REACH_THRESHOLD לא מוגדר	הוספה כ-static constexpr בהדר
7	worldToGrid, inBounds, isCellNavigable פרטיים	גישה דרך m_mapData הציבורי
8	ObjectEntry::updated/removed לא קיים	שינוי ל-TrackerFrame עם השדות הנכונים
9	in_queue_ מוגרש בהדר	שחזור השדה ב-DLite.hpp
10	chooseLocalGoal הוצהרה אך לא מומשה	הוספת מימוש מלא
11	MapLocal בנאי ב-NavigationManager	תיקון לאתחול עם m_mapData
12	getCells() נקרא מ-m_mapLocal	העברה ל-m_mapData->getCells()

#include "AStar.hpp" #include <algorithm> #include <limits> #include <cstring> #include <iostream> vector<uint64_t> AStar::run(unordered_map<uint64_t, Node>& graph, uint64_t start_id, uint64_t end_id) { if (graph.find(start_id)

pasted


#pragma once #include <vector> #include <string> #include <unordered_map> #include <unordered_set> #include <memory> #include <Eigen/Dense> #include "MapData.hpp" #include "ConfigNavigation.hpp" #include "GraphMap.hpp" class OctomapAdapter; struct TrackedObject; // --------------------

pasted


#pragma once #include <memory> #include <mutex> #include <atomic> #include <vector> #include <unordered_map> #include <optional> #include <chrono> #include "MapData.hpp" #include "MapLocal.hpp" #include "OctomapAdapter.hpp" #include "SensorFrame.hpp" #include "TrackedObject/TrackedOb

pasted


[main] Building folder: c:/Users/User/Desktop/TrackObject/build [build] Starting build [proc] Executing command: "C:\Program Files\CMake\bin\cmake.EXE" --build c:/Users/User/Desktop/TrackObject/build --config Debug --target ALL_BUILD -j 12 -- [build] MSBuild version 17.12.12+1cce77968 for .NET F

pasted

#include "GraphBuilder.hpp" // ← תיקון: חסר include עבור buildGraphFromPython / GraphResult
אבל אין לי קובץ כזה

אני רוצה לדעת איפה הפונקציה שמוציא ה מתוך המפה את המסלול ע"י הרצת אלגוריתם A*
אני מעלה את הקבצים של A* 
לצורך כך

#pragma once 
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <string>
#include <queue>
#include <cmath>
#include "GraphMap.hpp"
#define _USE_MATH_DEFINES
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

using namespace std;
constexpr double DEG_TO_RAD = M_PI / 180.0;

class AStar {
public:
    AStar() = default;
    vector<uint64_t> run(unordered_map<uint64_t, Node>& graph,
                         uint64_t start_id,
                         uint64_t end_id);    
    // פונקציה שמפעילה את החיפוש
    void FindPath(uint64_t start_id, uint64_t end_id);

private: 
    struct CompareNodes {
        bool operator()(Node* a, Node* b) { return a->f_value() > b->f_value(); }
    };  
    double CalculateHValue(const Node* node, const Node* goal) const;
    void   AddNeighbors(Node* current,Node* goal,unordered_map<uint64_t, Node>& graph,
            priority_queue<Node*,vector<Node*>,CompareNodes>& open_list);                                                                          
    vector<uint64_t> ConstructFinalPath(const Node* goal,
                                        const unordered_map<uint64_t, Node>& graph) const;
    static double toRadians(double deg) { return deg * DEG_TO_RAD; }

};
וכן אני מעלה את הקבצים שיש בהם שגיאות בעקבות השינויים

Show more
23:44
נתחיל בשתי השגיאות שנותרו:

שגיאה 1: inBounds ו-worldToGrid מוגדרים פעמיים ב-MapLocal.hpp — פעם ב-public ופעם ב-private.

שגיאה 2: GraphBuilder.hpp לא קיים — הפונקציה buildGraphFromPython כנראה מוגדרת במקום אחר בפרויקט שלך.

תיקון MapLocal.hpp — הסרת ההגדרות הכפולות מה-private:

cpp
#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <Eigen/Dense>
#include "MapData.hpp"
#include "ConfigNavigation.hpp"
#include "GraphMap.hpp"

class OctomapAdapter;
struct TrackedObject;

struct TrackerFrame {
    std::vector<int> updated;
    std::vector<int> removed;
};

struct ObjectEntry {
    int  track_id    = -1;
    bool is_dynamic  = false;
    int  last_row    = 0;
    int  last_col    = 0;
    int  body_r      = 0;
    int  inflation_r = 0;
    std::vector<int> painted_indices;
};

class MapLocal {
public:
    MapLocal() = default;
    explicit MapLocal(std::shared_ptr<MapData> data);

    void setMapData(std::shared_ptr<MapData> data) {
        m_data = std::move(data);
    }

    // ── delegate ל-MapData (ציבורי) ──────────────────────────────
    bool worldToGrid(float wx, float wy, int& col, int& row) const {
        return m_data->worldToGrid(wx, wy, col, row);
    }
    bool inBounds(int row, int col) const {
        return m_data->inBounds(row, col);
    }
    bool isCellNavigable(int col, int row) const {
        return m_data->isCellNavigable(col, row);
    }

    void shiftOrigin(float new_robot_x, float new_robot_y);
    void buildFromOctomap();
    void updateObjects(
        const TrackerFrame& batch,
        std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObject
    );
    void inflationPass();
    void buildRouteLimitLayer(
        const std::vector<uint64_t>& path_node_ids,
        const std::unordered_map<uint64_t, Node>& graph
    );
    void clearRouteLimitLayer();
    void fuseMaps();

    std::vector<CellUpdate> getCells() { return this->cells; }

    uint8_t getCostForPlanner(float wx, float wy) const;
    uint8_t getCostForPlanner(int row, int col)   const;
    uint8_t getCost      (float wx, float wy) const;
    float   getConfidence(float wx, float wy) const;
    bool    isNavigable  (float wx, float wy) const { return getCost(wx, wy) < COST_INFLATED; }

    float getOriginX() const { return m_data->originX; }
    float getOriginY() const { return m_data->originY; }

    const std::array<uint8_t, GRID_W * GRID_H>& getFusedMap()    const { return m_data->costMapFused;    }
    const std::array<uint8_t, GRID_W * GRID_H>& rawLidarLayer()  const { return m_data->lidarLayer;      }
    const std::array<uint8_t, GRID_W * GRID_H>& rawObjectLayer() const { return m_data->objectLayer;     }
    const std::array<uint8_t, GRID_W * GRID_H>& rawRouteLayer()  const { return m_data->routeLimitLayer; }

    void registerCellChange(int row, int col, uint8_t new_cost);

private:
    std::shared_ptr<MapData> m_data;
    std::unordered_map<int, ObjectEntry> m_objectRegistry;
    std::vector<CellUpdate> cells;

    // ── כלים פנימיים — ללא delegate, עובדים ישירות על m_data ────
    // ← תיקון: הוסרו inBounds ו-worldToGrid הכפולים מה-private
    //          flatIdx נשאר כי הוא לא מוגדר ב-MapData
    inline int flatIdx(int row, int col) const { return row * GRID_W + col; }

    void markEdgeCorridor(
        const Eigen::Vector2f& from_pos,
        const Eigen::Vector2f& to_pos,
        float corridor_half_width
    );
    void applyNodeCurbRestrictions(
        const std::vector<uint64_t>& path_node_ids,
        const std::unordered_map<uint64_t, Node>& graph
    );
    static float distancePointToSegment(
        const Eigen::Vector2f& p,
        const Eigen::Vector2f& a,
        const Eigen::Vector2f& b
    );
    static const Edge* findEdgeBetween(
        const std::unordered_map<uint64_t, Node>& graph,
        uint64_t from_id,
        uint64_t to_id
    );
    void paintCircleOnRouteLayer(
        const Eigen::Vector2f& center_pos,
        float radius_m,
        uint8_t cost
    );
    void eraseObjectFootprint(ObjectEntry& entry);
    void paintObject(const TrackedObject& obj, ObjectEntry& entry);
    void markBodyOnObjectLayer(
        int rowC, int colC,
        int body_r, int inflation_r,
        bool isStaticConfirmed, float obj_confidence,
        std::unordered_set<int>& painted_set
    );
    void applyDynamicForwardField(
        int rowC, int colC,
        const Eigen::Vector3f& vel,
        float speed, float obj_confidence,
        std::unordered_set<int>& painted_set
    );
    void inflateObjectCell(
        int rowC, int colC, int r,
        std::unordered_set<int>& painted_set
    );
    static int securitySpace(const std::string& type_label);
};
תיקון NavigationManager.hpp — הסרת #include "GraphBuilder.hpp" והעברת ה-forward declaration:

cpp
#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <optional>
#include <chrono>

#include "MapData.hpp"
#include "MapLocal.hpp"
#include "OctomapAdapter.hpp"
#include "SensorFrame.hpp"
#include "TrackedObject/TrackedObject.hpp"
#include "DLite.hpp"
#include "AStar.hpp"
#include "GraphMap.hpp"
#include "Location.hpp"
// ← תיקון: הוסר #include "GraphBuilder.hpp" — הקובץ לא קיים.
//           buildGraphFromPython ו-GraphResult מוכרזים כ-forward declarations
//           ו-include האמיתי יהיה רק ב-.cpp

struct GraphResult;   // ← forward declaration בלבד

static constexpr float NODE_REACH_THRESHOLD = 3.0f;

class NavigationManager {
public:
    static NavigationManager& getInstance() {
        static NavigationManager instance;
        return instance;
    }

    std::shared_ptr<MapData> getMapData() const { return m_mapData; }

    void createMapGlobal(Location myplace, Location mygoal);
    void tryAdvanceGlobalNode(const Eigen::VectorXd& state);
    const Node& getNodeById(uint64_t id) const;
    std::vector<uint64_t> getPathGlobal() const;
    std::vector<uint64_t> getPateGlobal() const { return getPathGlobal(); }

    void updateFromLidar(
        const std::shared_ptr<FramePointers>& frame,
        float robot_x, float robot_y
    );
    void updateFromObjects(
        const ObjectEntry& batch,
        std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects
    );
    void finalize();

    void runDLite(
        const std::shared_ptr<FramePointers>& frame,
        const std::unordered_map<int, std::shared_ptr<TrackedObject>>& trackedObjects,
        const Eigen::VectorXd& state
    );

    void setDynamicBlocked(bool blocked);
    void setTrafficBlocked(bool blocked);
    void setCrossingBlocked(bool blocked);
    bool shouldStop() const;

    bool stoppedByDynamic()  const { return m_dynamicBlocked.load();  }
    bool stoppedByTraffic()  const { return m_trafficBlocked.load();  }
    bool stoppedByCrossing() const { return m_crossingBlocked.load(); }

    void updatePath(std::vector<std::pair<int,int>> newPath);
    std::optional<std::pair<int,int>> getNextStep();
    void popNextStep();
    std::vector<std::pair<int,int>> getPathLiveTime() const;

    Node* getCurrentNode() const;
    Edge* getCurrentEdge() const;
    Location getLocation() const { return myGoal; }

private:
    NavigationManager()
        : m_mapData(std::make_shared<MapData>())
        , m_mapLocal(m_mapData)
    {}

    NavigationManager(const NavigationManager&)            = delete;
    NavigationManager& operator=(const NavigationManager&) = delete;

    void updateCurrentEdge();
    bool chooseLocalGoal(float robot_x, float robot_y,
                         int robot_col, int robot_row,
                         int& goal_col, int& goal_row);
    bool chooseLookaheadGoal(float robot_x, float robot_y,
                              int robot_col, int robot_row,
                              int& goal_col, int& goal_row);
    bool goalChanged(int goal_col, int goal_row) const;
    bool findAlternativeLocalGoal(int goal_col, int goal_row,
                                  int& alt_col, int& alt_row);

    std::shared_ptr<MapData> m_mapData;
    OctomapAdapter           m_octomap;
    MapLocal                 m_mapLocal;
    DLite                    m_dlite;
    AStar                    m_astar;

    bool m_dliteInitialized = false;
    int  m_lastGoalCol      = -1;
    int  m_lastGoalRow      = -1;

    std::atomic_bool m_dynamicBlocked  {false};
    std::atomic_bool m_trafficBlocked  {false};
    std::atomic_bool m_crossingBlocked {false};

    std::chrono::steady_clock::time_point m_lastDynamicDangerTime =
        std::chrono::steady_clock::time_point::min();

    std::vector<std::pair<int,int>> m_currentPath;
    mutable std::mutex              m_pathMutex;

    std::unordered_map<uint64_t, Node> myMap;
    std::vector<uint64_t>              m_globalPath;
    std::size_t                        m_globalPathIndex = 0;

    Node* m_currentNode = nullptr;
    Edge* m_currentEdge = nullptr;

    Location myGoal;
};
תיקון NavigationManager.cpp — מוסיפים את ה-include האמיתי של GraphResult וה-buildGraphFromPython רק כאן:

בתחילת הקובץ, מחפשים את הקובץ שמגדיר buildGraphFromPython. זו פונקציה שמדברת עם Python — היא כנראה נמצאת אצלך באחד השמות האלה: PythonBridge.hpp, GraphLoader.hpp, OSMLoader.hpp, או משהו דומה. בינתיים מחליפים את השימוש כך:

cpp
#include "NavigationManager.hpp"
#include "MapLive/OctomapAdapter.hpp"
// ← תיקון: במקום GraphBuilder.hpp — מחפשים את הקובץ האמיתי שמגדיר
//           GraphResult ו-buildGraphFromPython אצלך בפרויקט.
//           נניח שהוא נקרא PythonBridge.hpp — מחליפים לשם האמיתי:
// #include "PythonBridge.hpp"   ← בטל הערה וכתוב את השם הנכון

// ── struct זמני עד שתמצאי את הקובץ הנכון ────────────────────────
// אם buildGraphFromPython מוגדר אצלך בקובץ אחר, תוסיפי include לו כאן
struct GraphResult {
    std::unordered_map<uint64_t, Node> graph;
    uint64_t source_node = 0;
    uint64_t target_node = 0;
};

// forward declaration — המימוש חייב להיות בקובץ אחר שמקמפל עם הפרויקט
GraphResult buildGraphFromPython(double src_lat, double src_lon,
                                  double dst_lat, double dst_lon,
                                  const std::string& config);
// ─────────────────────────────────────────────────────────────────
לגבי שאלתך על A:* הפונקציה שמריצה את A* ומחזירה את המסלול היא AStar::run() שקיימת אצלך ב-AStar.cpp. היא נקראת מתוך createMapGlobal בשורה:

cpp
m_globalPath = m_astar.run(myMap, res.source_node, res.target_node);
הפונקציה מקבלת את הגרף, מזהה מוצא ומזהה יעד, ומחזירה vector<uint64_t> — רשימת מזהי צמתים מהמוצא ליעד. הנה זרימת הלוגיקה שלה:

run()
  ├── איפוס כל הצמתים בגרף (g=∞, parent=null)
  ├── מגדיר g=0 ל-source, מחשב h לפי Haversine
  ├── לולאה: שולף את הצומת עם f=g+h הנמוכה ביותר
  │     ├── אם הגענו ל-target → ConstructFinalPath()
  │     └── AddNeighbors() → מוסיף שכנים שלא ביקרנו
  └── אם הלולאה נגמרה ללא יעד → מחזיר {}
כדי לחבר את GraphResult אל הפרויקט בצורה נכונה, תחפשי בפרויקט שלך קובץ שמכיל את המילה buildGraphFromPython ותגידי לי מה שמו — נוסיף את ה-include הנכון.







Claude is AI and can make mistakes. Please double-check responses.
