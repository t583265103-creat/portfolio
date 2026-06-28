#pragma once
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <memory>
#include "TrackedObject.hpp"
#include "SensorFrame.hpp"


class IcpEstimator {
public:

    struct IcpResult {
        Eigen::Matrix4f transformation;
        double fitness_score;
        bool is_valid;
    };

    IcpEstimator() = default;   
    IcpResult process(std::shared_ptr<FramePointers> fram);

    private:
    // מאחדת את ענני הנקודות של כל האובייקטים לענן אחד מאוחד
    pcl::PointCloud<pcl::PointXYZI>::Ptr createNewCloud(const std::vector<std::shared_ptr<TrackedObject>>& best_objs);
    
    // מבצעת ICP בין ענן מקור לענן מטרה ומחזירה את מטריצת הטרנספורמציה (4x4)
    IcpResult applyICP(pcl::PointCloud<pcl::PointXYZI>::Ptr source, 
                             pcl::PointCloud<pcl::PointXYZI>::Ptr target);
};