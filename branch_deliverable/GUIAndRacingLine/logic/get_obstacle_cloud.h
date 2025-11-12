//
// Created by Vi De Matteis on 11/11/25.
//



#ifndef SECOND_PRJ_SETUP_GET_OBSTACLE_CLOUD_H
#define SECOND_PRJ_SETUP_GET_OBSTACLE_CLOUD_H

pcl::PointCloud<pcl::PointXYZ>::Ptr
get_obstacle_cloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cones_cloud
);



#endif //SECOND_PRJ_SETUP_GET_OBSTACLE_CLOUD_H