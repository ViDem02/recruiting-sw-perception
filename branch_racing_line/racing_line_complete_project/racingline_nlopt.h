//
// Created by Vi De Matteis on 10/11/25.
//

#ifndef RACINGLINE_RACINGLINE_NLOPT_H
#define RACINGLINE_RACINGLINE_NLOPT_H
#include <pcl/point_cloud.h>

namespace pcl
{
    struct PointXYZ;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
get_racing_line_point_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cones,
    double robot_x,
    double robot_y,
    float y_constant = 0,
    int nr_points_lin_space = 100
    );

#endif //RACINGLINE_RACINGLINE_NLOPT_H
