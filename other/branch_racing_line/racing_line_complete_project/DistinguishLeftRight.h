#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <limits>

namespace cones {

struct Result {
    pcl::PointCloud<pcl::PointXYZ>::Ptr left;
    pcl::PointCloud<pcl::PointXYZ>::Ptr right;
};

Result distinguishLeftRight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cones,
    double robot_x, double robot_y,
    double angle_low_deg = -45.0, double angle_high_deg = 45.0,
    double max_distance = std::numeric_limits<double>::infinity()
    );

}
