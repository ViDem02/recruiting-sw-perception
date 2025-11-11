//
// Created by Vi De Matteis on 11/11/25.
//

#ifndef SECOND_PRJ_SETUP_CONES_REGOC_H
#define SECOND_PRJ_SETUP_CONES_REGOC_H
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

std::vector<int> detectCones(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<pcl::PointIndices> &cluster_indices,
    const std::string &cone_model_ply,
    int icp_max_iter = 40,
    double fitness_threshold = 0.01,
    double height_threshold = -0.01, // <-- adjust as needed (meters)
    double min_cone_height = 0.3, // <-- minimum expected cone height in meters
    double max_cone_height = 0.5,
    float max_yaw_deg = 20.0f,
    bool visualize = true,
    bool verbose = false);

#endif //SECOND_PRJ_SETUP_CONES_REGOC_H