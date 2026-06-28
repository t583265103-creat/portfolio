#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

// הוספת הדרים חסרים עבור הטיפוסים שבשימוש המחלקה
#include <yaml-cpp/yaml.h>
#include "SensorProcessing.hpp"
#include "LidarObject.hpp"
#include "SensorConfig.hpp"
#include "ROI_box.hpp"

// ירושה נכונה: הקלט הוא וקטור של עננים, הפלט הוא וקטור של אובייקטים
class LidarProcessing : public SensorProcessing<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, std::vector<LidarObject>> {
public:
    LidarProcessing() = default; 
    std::vector<LidarObject> process(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds,double tss) override;
    pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalCloud() const { return globalCloud; }

private:
    void processLidar(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& raw_clouds);
    
    // פונקציה ליצירת ענן מאינדקסים (שימוש פנימי)
    pcl::PointCloud<pcl::PointXYZI>::Ptr createcluod(pcl::PointIndices indices);
    
    void extractGeometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, LidarObject& LO);
    float calculateConfidence(const LidarObject& LO, const ROI_box& params);
    
    // מימוש ה-Template: יצירת אובייקט בודד מתוך ענן בודד
    LidarObject createObject(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void LoadingConfiglidar(const YAML::Node& node);
    
    std::vector<pcl::PointIndices> output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalCloud;
    ROI_box roi;
    SensorLidarConfig Config;
    double ts;
};