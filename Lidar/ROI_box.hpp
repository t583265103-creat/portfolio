// #pragma once
// #include <Eigen/Dense>
// #include <yaml-cpp/yaml.h>

// struct SensorLidarConfig {
//     Eigen::Matrix3f rotation;
//     Eigen::Vector3f translation;
// };
// struct ROI_box {
//     // גבולות גזרה (x, y, z, w)
//     Eigen::Vector4f minPt;
//     Eigen::Vector4f maxPt;

//     // פרמטרים לעיבוד
//     float voxelSize;
//     int minNeighbors;
//     float searchRadius;
//     float minIntensity;
//     float maxDistanceBetweenPoints;

//     // ספירת נקודות
//     int minPointsInCloud;
//     int maxPointsInCloud;

//     ROI_box() :
//         minPt(-50.0f, -20.0f, -2.0f, 1.0f),
//         maxPt(50.0f, 20.0f, 2.0f, 1.0f),
//         voxelSize(0.1f),
//         minNeighbors(5),
//         searchRadius(0.2f),
//         minIntensity(10.0f),
//         maxDistanceBetweenPoints(0.3f),
//         minPointsInCloud(10),
//         maxPointsInCloud(5000)
//     {
//     }
// }; 