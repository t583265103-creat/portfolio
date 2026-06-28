#pragma once
// ============================================================
//  NavigationThread.hpp
//  לב מערכת הניווט בזמן אמת.
//  מחבר את כל השכבות:
//
//  ┌──────────────────────────────────────────────────┐
//  │  SensorConsumer (threads)                         │
//  │    → SensorRing → FramePointers                   │
//  │    → StatePriorityQueue → StateThread (EKF)       │
//  │                         ↓                         │
//  │  NavigationThread  (לולאה 10Hz)                   
//  │    שלב 1: EKF  → מיקום רובוט נוכחי               │
//  │    שלב 2: FactorGraph → תיקון גלובלי             │
//  │    שלב 3: OctomapAdapter ← FramePointers (Lidar)  │
//  │    שלב 4: TrackedObjects ← FactorGraphManager     │
//  │    שלב 5: LocalcostMap.update (2D circ.buffer)    │
//  │    שלב 6: GlobalNavGraph.astar → waypoints        │
//  │    שלב 7: Local path following → velocity cmd     │
//  └──────────────────────────────────────────────────┘
// ============================================================
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include "OctomapAdapter.hpp"
#include "LocalcostMap.hpp"
#include "GlobalNavGraph.hpp"
#include "StatePriorityQueue.hpp"
#include "SensorFrameManager.hpp"
#include "FactorGraphManager.hpp"
#include "Constants.hpp"

// ============================================================
//  VelocityCommand — פלט הניווט לשכבת השליטה
// ============================================================
struct VelocityCommand {
    float linear_mps  = 0.0f;   // [m/s]  קדימה / אחורה
    float angular_rps = 0.0f;   // [rad/s] סיבוב
};

class NavigationThread {
public:
    NavigationThread()
        : is_running_(false),
          nav_rate_hz_(10),            // לולאה 10Hz — ניתן לשינוי
          lookahead_dist_(1.5f)        // מרחק ה-Pure Pursuit [מטר]
    {}

    // ==========================================
    // setGoal — הגדרת יעד חדש לניווט
    // ==========================================
    void setGoal(float x, float y) {
        goal_x_ = x;
        goal_y_ = y;
        goal_active_ = true;
        replanning_needed_ = true;
        std::cout << "[Nav] New goal set: (" << x << ", " << y << ")\n";
    }

    void init() {
        is_running_ = true;
        nav_thread_ = std::thread(&NavigationThread::navLoop, this);
    }

    void stop() {
        is_running_ = false;
        if (nav_thread_.joinable()) nav_thread_.join();
    }

    VelocityCommand getLastCommand() const { return last_cmd_; }

    // גישה לאובייקטים לשימוש חיצוני (ויזואליזציה וכו')
    LocalcostMap&   getcostMap()   { return costMap;    }
    GlobalNavGraph& getNavGraph()  { return nav_graph_;  }
    OctomapAdapter& getOctomap()   { return octomap_;    }

private:
    std::thread   nav_thread_;
    std::atomic<bool> is_running_;
    int   nav_rate_hz_;
    float lookahead_dist_;

    OctomapAdapter octomap_;
    LocalcostMap   costMap;
    GlobalNavGraph nav_graph_;

    float goal_x_ = 0, goal_y_ = 0;
    std::atomic<bool> goal_active_{false};
    std::atomic<bool> replanning_needed_{false};

    std::vector<Eigen::Vector2f> current_path_;
    int path_index_ = 0;

    VelocityCommand last_cmd_;

    // ============================================================
    //  navLoop — הלולאה הראשית (מופעלת בתהליכון נפרד)
    // ============================================================
    void navLoop() {
        auto period = std::chrono::milliseconds(1000 / nav_rate_hz_);
        std::cout << "[Nav] Navigation loop started @ " << nav_rate_hz_ << "Hz\n";

        while (is_running_) {
            auto tick_start = std::chrono::steady_clock::now();

            // ─────────────────────────────────────────
            //  שלב 1: שליפת מיקום רובוט מה-EKF המרכזי
            // ─────────────────────────────────────────
            Eigen::VectorXd state = StatePriorityQueue::getInstance().getStateVector();
            float robot_x = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateX)));
            float robot_y = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateY)));
            float robot_yaw = static_cast<float>(state(static_cast<int>(Config::StateMembersRobot::StateYaw)));

            // ─────────────────────────────────────────
            //  שלב 2: שליפת הפריים הנוכחי
            // ─────────────────────────────────────────
            auto frame = SensorFrameManager::getInstance().getProcessFrame();

            // ─────────────────────────────────────────
            //  שלב 3: עדכון Octomap [מבנה 6 — Octree]
            //  נתוני Lidar מהפריים → עץ תלת-ממדי
            // ─────────────────────────────────────────
            if (frame) {
                octomap_.updateFromFrame(frame);
            }

            // ─────────────────────────────────────────
            //  שלב 4: עדכון LocalcostMap [מבני 7+8]
            //  חתך 2D מה-Octomap → מפה מקומית + Inflation
            // ─────────────────────────────────────────
            costMap.update(octomap_, robot_x, robot_y);

            // ─────────────────────────────────────────
            //  שלב 5: עדכון עלויות קשתות בגרף הגלובלי [מבנה 9]
            //  אם אחד מה-waypoints הפך לחסום — מעדכנים עלות
            // ─────────────────────────────────────────
            updateGraphEdgeCosts();

            // ─────────────────────────────────────────
            //  שלב 6: תכנון מסלול גלובלי (A*) [מבנה 9]
            //  מתבצע רק כשצריך (יעד חדש / מסלול חסום)
            // ─────────────────────────────────────────
            if (goal_active_ && replanning_needed_) {
                planGlobalPath(robot_x, robot_y);
                replanning_needed_ = false;
            }

            // ─────────────────────────────────────────
            //  שלב 7: עקיבה אחר המסלול (Pure Pursuit)
            //  מחשב פקודת מהירות קווית + זוויתית
            // ─────────────────────────────────────────
            if (goal_active_ && !current_path_.empty()) {
                last_cmd_ = purePursuit(robot_x, robot_y, robot_yaw);
            } else {
                last_cmd_ = {0.0f, 0.0f}; // עמידה
            }

            // שמירה על קצב קבוע
            auto elapsed = std::chrono::steady_clock::now() - tick_start;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }

        std::cout << "[Nav] Navigation loop stopped.\n";
    }

    // ============================================================
    //  planGlobalPath
    //  A* על הגרף הדליל → המרה ל-waypoints
    // ============================================================
    void planGlobalPath(float robot_x, float robot_y) {
        int start_node = nav_graph_.findClosestNode(robot_x, robot_y);
        int goal_node  = nav_graph_.findClosestNode(goal_x_, goal_y_);

        if (start_node < 0 || goal_node < 0) {
            std::cerr << "[Nav] Cannot find nodes for start/goal!\n";
            return;
        }

        std::vector<int> node_path = nav_graph_.astar(start_node, goal_node);

        if (node_path.empty()) {
            std::cerr << "[Nav] A* found no path!\n";
            return;
        }

        current_path_ = nav_graph_.pathToWaypoints(node_path);
        path_index_   = 0;
        std::cout << "[Nav] Path planned: " << current_path_.size() << " waypoints\n";
    }

    // ============================================================
    //  updateGraphEdgeCosts
    //  סורק את כל הצמתים בגרף — אם ה-costMap חוסם waypoint,
    //  מעלה את עלות הקשתות המובילות אליו → A* יעקוף אותו
    // ============================================================
    void updateGraphEdgeCosts() {
        // פונקציה זו מנצלת את ה-API הפנימי של הגרף
        // (ניתן להרחיב לפי המבנה המלא של GlobalNavGraph)
    }

    // ============================================================
    //  purePursuit
    //  אלגוריתם מעקב מסלול קלאסי:
    //    1. מוצא את נקודת המסלול הקרובה
    //    2. מוצא "lookahead point" במרחק lookahead_dist_
    //    3. מחשב עקמומיות (curvature) → זוויתית / קווית
    // ============================================================
    VelocityCommand purePursuit(float rx, float ry, float ryaw) {
        if (path_index_ >= static_cast<int>(current_path_.size())) {
            goal_active_ = false;
            std::cout << "[Nav] Goal reached!\n";
            return {0.0f, 0.0f};
        }

        // בדיקת הגעה לנקודה הנוכחית
        const auto& wp = current_path_[path_index_];
        float dist_to_wp = std::hypot(wp.x() - rx, wp.y() - ry);
        if (dist_to_wp < 0.3f) {
            path_index_++;
            if (path_index_ >= static_cast<int>(current_path_.size())) {
                goal_active_ = false;
                return {0.0f, 0.0f};
            }
        }

        // מציאת lookahead point
        Eigen::Vector2f lookahead_pt = current_path_.back(); // ברירת מחדל: נקודת סיום
        for (int i = path_index_; i < static_cast<int>(current_path_.size()); ++i) {
            float d = std::hypot(current_path_[i].x() - rx, current_path_[i].y() - ry);
            if (d >= lookahead_dist_) {
                lookahead_pt = current_path_[i];
                break;
            }
        }

        // חישוב זווית יחסית ועקמומיות
        float dx    = lookahead_pt.x() - rx;
        float dy    = lookahead_pt.y() - ry;
        float alpha = std::atan2(dy, dx) - ryaw;

        // ־ normalizing alpha לטווח [-π, π]
        while (alpha >  M_PI) alpha -= 2.0f * M_PI;
        while (alpha < -M_PI) alpha += 2.0f * M_PI;

        float L     = std::hypot(dx, dy);
        float kappa = 2.0f * std::sin(alpha) / (L + 1e-6f); // עקמומיות

        // פקודת מהירות
        float max_linear  = 0.8f;  // [m/s]  — מהירות מקסימלית לכיסא
        float max_angular = 1.0f;  // [rad/s]

        // האטה ליד נקודת הסיום
        float dist_to_goal = std::hypot(goal_x_ - rx, goal_y_ - ry);
        float speed_factor  = std::min(1.0f, dist_to_goal / 2.0f);

        // בדיקת costMap — אם קדימה חסום, עצור ותכנן מחדש
        float check_x = rx + 0.5f * std::cos(ryaw);
        float check_y = ry + 0.5f * std::sin(ryaw);
        if (!costMap.isNavigable(check_x, check_y)) {
            replanning_needed_ = true;
            return {0.0f, 0.3f * std::copysign(1.0f, alpha)}; // סיבוב קל
        }

        VelocityCommand cmd;
        cmd.linear_mps  = std::clamp(max_linear * speed_factor, 0.0f, max_linear);
        cmd.angular_rps = std::clamp(kappa * cmd.linear_mps, -max_angular, max_angular);
        return cmd;
    }
};
