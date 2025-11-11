//
// Created by Vi De Matteis on 11/11/25.
//

#include <iostream>
#include <thread>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


#include "get_obstacle_cloud.h"

#include <pcl/segmentation/segment_differences.h>

#include "get_cones_cloud.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr
get_obstacle_cloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cones_cloud
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr obst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*src_cloud, *obst_cloud);

    pcl::SegmentDifferences<pcl::PointXYZ> seg_diff;
    seg_diff.setInputCloud(obst_cloud);
    seg_diff.setTargetCloud(cones_cloud);
    seg_diff.setDistanceThreshold(1e-6); // exact match threshold
    seg_diff.segment(*obst_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(obst_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.68, 1);
    pass.filter(*obst_cloud);


    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    pcl::ExtractIndices<pcl::PointXYZ> reg_grow_extract;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::IndicesPtr indices(new std::vector<int>);
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    pcl::removeNaNFromPointCloud(*obst_cloud, *indices);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(obst_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    reg.setInputCloud(obst_cloud);
    reg.setMinClusterSize(500);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(300);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(1 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1);
    reg.extract(clusters);
    reg_grow_extract.setInputCloud(obst_cloud);
    for (const auto &cluster: clusters)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(cluster));
        reg_grow_extract.setIndices(cluster_ptr);
        reg_grow_extract.setNegative(true);
        reg_grow_extract.setKeepOrganized(true);
        reg_grow_extract.filter(*obst_cloud);
    }


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(obst_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(10);
    sor.filter(*obst_cloud);

    return obst_cloud;
}
