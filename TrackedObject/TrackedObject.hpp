#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp> // הוספת אינקלוד עבור cv::Point2f
#include <string>
#include <vector>
#include "KalmanFilter/ekf.hpp"
#include "SensorObject.hpp"
#include "Constants.hpp"


struct TrackedObject : public SensorObject {
    // ==========================================
    // 1. זיהוי ומטא-דאטה (Identity)
    // ==========================================
    std::string type_label;
    std::string traffic_light_color;
    int frames_active;
    int missed_frames_counter;
    bool updated_this_frame;
    // ==========================================
    // 2. וקטור מצב ומטריצות קלמן (Kalman State)
    // ==========================================
    // ה-EKF מחזיק כעת את וקטור המצב (filter.x) ואת מטריצת האי-וודאות (filter.P)
    EKF<Config::StateSizeObject> filter;
    Eigen::MatrixXd F;

    // ==========================================
    // 3. גאומטריה ומבנה (Geometry)
    // ==========================================
    float length, width, height;
    float curvature;        // קיעור פני השטח
    Eigen::Vector4f normal; // וקטור נורמל ממוצע

    // ==========================================
    // 4. נתונים גולמיים ומסלול (Data & History)
    // ==========================================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; // ענן הנקודות הנוכחי
    std::vector<cv::Point2f> image_points;      // רשימת נקודות מהתמונה
    cv::Rect bounding_box;
    std::vector<Eigen::Vector3f> path;          // היסטוריית מיקומים (לזנב)
    Eigen::Matrix3f point_cloud_covariance;     // פיזור הנקודות (PCA)

    // ==========================================
    // 5. בנאי ברירת מחדל (Default Constructor)
    // ==========================================
    TrackedObject()
        : SensorObject( Config::StateSizeObject, Config::MeasurementSizeSensorObgect,SensorType::TrackedObject,false,false,false), // קריאה לבנאי של מחלקת האב
          type_label("unknown"),
          traffic_light_color("unknown"),
          frames_active(0),
          missed_frames_counter(0),
          updated_this_frame(false),
          length(0.0f),
          width(0.0f),
          height(0.0f),
          curvature(0.0f),
          bounding_box(0, 0, 0, 0),
          cloud(new pcl::PointCloud<pcl::PointXYZI>()) 
    {
        // אתחול פנימי של ה-Kalman Filter
        filter.x.setZero();
        filter.P.setIdentity();
        F.setZero();
        // איפוס מטריצות ווקטורים נוספים
        normal.setZero();
        point_cloud_covariance.setZero();
        
        // אתחול משתנה ה-confidence ששייך למחלקת האב
        confidence = 0.0f;
        
        // ניקוי וקטורים
        image_points.clear();
        path.clear();
    }

    // ==========================================
    // 6. פונקציות עזר (Utility Functions)
    // ==========================================
    
    // שליפת מיקום נוכחי מתוך וקטור המצב של הפילטר
    Eigen::Vector3f position() const { 
        return filter.x.template head<3>().cast<float>(); 
    }
    
    // שליפת מהירות נוכחית מתוך וקטור המצב של הפילטר
    Eigen::Vector3f velocity() const override{ 
        return filter.x.template segment<3>(3).cast<float>(); 
    }
    // חישוב מהירות בקמ"ש
    float get_speed_kph() const {
        Eigen::Vector3f vel = velocity();
        return vel.norm() * 3.6f;
    }

    // חישוב תאוצה (בהנחה ווקטור המצב בגודל 9 ומכיל תאוצה ב-3 האיברים האחרונים)
    float get_acceleration_norm() const {
        if (Config::StateSizeObject >= 9) {
            Eigen::Vector3f acc = filter.x.template tail<3>().cast<float>();
            return acc.norm();
        }
        return 0.0f;
    }

    // כיוון נסיעה (זווית ברדיאנים)
    float get_travel_yaw() const {
        Eigen::Vector3f vel = velocity();
        return std::atan2(vel.y(), vel.x());
    }

    // אתחול אובייקט חדש
    void init(const Eigen::Vector3f& pos, float initial_confidence) {
        filter.x.setZero();
        filter.x.template head<3>() = pos.cast<double>(); // השמה של המיקום הראשוני בתוך הפילטר
        
        // אתחול מטריצת P עם אי-וודאות ראשונית
        filter.P = Eigen::Matrix<double, Config::StateSizeObject, Config::StateSizeObject>::Identity() * 1.0;
        
        // נותנים אי-וודאות גבוהה יותר למהירות ותאוצה בהתחלה (מאינדקס 3 ואילך)
        if (Config::StateSizeObject > 3) {
            filter.P.template block<Config::StateSizeObject - 3, Config::StateSizeObject - 3>(3, 3) *= 10.0;
        }
        
        confidence = initial_confidence;
        path.push_back(pos);
    }
};