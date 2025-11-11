#include "DistinguishLeftRight.h"
#include <cmath>
#include <algorithm>

namespace cones {

Result distinguishLeftRight(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& src_cones,
    double robot_x, double robot_y,
    double angle_low_deg, double angle_high_deg,
    double max_distance)
{
    Result res;
    
    // Filter cones by distance - defensive clamp for negative max_distance
    pcl::PointCloud<pcl::PointXYZ>::Ptr cones(new pcl::PointCloud<pcl::PointXYZ>());
    
    double md = std::max(0.0, max_distance);
    double max_dist_sq = md * md;
    
    // Transform coordinates and filter by distance
    for (size_t i = 0; i < src_cones->size(); ++i)
    {
        pcl::PointXYZ p;
        // Coordinate transformation: swap x and z
        p.x = src_cones->points[i].z;
        p.z = src_cones->points[i].x;
        p.y = 0;
        
        double dx = p.x - robot_x;
        double dy = p.y - robot_y;
        double dist_sq = dx*dx + dy*dy;
        
        if (dist_sq <= max_dist_sq) {
            cones->push_back(p);
        }
    }
    
    // Convert angle thresholds to radians
    double angle_low_rad = angle_low_deg * M_PI / 180.0;
    double angle_high_rad = angle_high_deg * M_PI / 180.0;
    
    // Distinguish left and right based on angle from robot
    for (const auto& cone : cones->points)
    {
        double dx = cone.x - robot_x;
        double dy = cone.y - robot_y;
        double angle = std::atan2(dy, dx);
        
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        
        if (angle >= angle_low_rad && angle <= angle_high_rad) {
            // Within forward cone - distinguish left/right by cross product
            if (dy > 0) {
                res.left_cones->push_back(cone);
            } else {
                res.right_cones->push_back(cone);
            }
        }
    }
    
    return res;
}

} // namespace cones
