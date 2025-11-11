#ifndef DISTINGUISH_LEFT_RIGHT_H
#define DISTINGUISH_LEFT_RIGHT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>

namespace cones {

struct Result {
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cones;
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cones;
    
    Result() 
        : left_cones(new pcl::PointCloud<pcl::PointXYZ>()),
          right_cones(new pcl::PointCloud<pcl::PointXYZ>()) {}
};

/**
 * Distinguishes left and right cones based on their position relative to the robot.
 * 
 * @param cones Input point cloud of cone centers
 * @param robot_x X position of the robot
 * @param robot_y Y position of the robot
 * @param angle_low_deg Lower angle threshold in degrees (default: -45.0)
 * @param angle_high_deg Upper angle threshold in degrees (default: 45.0)
 * @param max_distance Maximum distance from robot to consider cones (default: infinity, i.e., no filtering)
 * @return Result structure containing left and right cone point clouds
 */
Result distinguishLeftRight(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cones,
    double robot_x, double robot_y,
    double angle_low_deg = -45.0, double angle_high_deg = 45.0,
    double max_distance = std::numeric_limits<double>::infinity()
);

} // namespace cones

#endif // DISTINGUISH_LEFT_RIGHT_H
