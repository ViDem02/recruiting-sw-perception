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
#include "get_cones_cloud.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include "cones_recog.h"





void removeWalls(const pcl::PointCloud<pcl::PointXYZ>::Ptr &dst_cloud,
                 std::vector<pcl::PointIndices> &eucl_cluster_indxes)
{
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    pcl::ExtractIndices<pcl::PointXYZ> reg_grow_extract;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::IndicesPtr indices(new std::vector<int>);
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    pcl::removeNaNFromPointCloud(*dst_cloud, *indices);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(dst_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    reg.setInputCloud(dst_cloud);
    reg.setMinClusterSize(500);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(300);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(1 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1);
    reg.extract(clusters);
    reg_grow_extract.setInputCloud(dst_cloud);
    for (const auto &cluster: clusters)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(cluster));
        reg_grow_extract.setIndices(cluster_ptr);
        reg_grow_extract.setNegative(true);
        reg_grow_extract.setKeepOrganized(true);
        reg_grow_extract.filter(*dst_cloud);
    }


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> eucl_cluster_extr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(dst_cloud);

    eucl_cluster_extr.setClusterTolerance(0.05);
    eucl_cluster_extr.setMinClusterSize(8);
    eucl_cluster_extr.setMaxClusterSize(500);
    eucl_cluster_extr.setSearchMethod(tree2);
    eucl_cluster_extr.setInputCloud(dst_cloud);
    eucl_cluster_extr.extract(eucl_cluster_indxes);
}

ConeDetectionResults
get_cones_cloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
    double initial_sor_std_dev_thr,
    const float cut_bottom,
    const float cut_top,
    const std::string& cone_model_name,
    double detect_cones_fitness_detection, bool verbose, const pcl::PointCloud<pcl::PointXYZ>::Ptr &ideal_cone_model, bool visualize_cone_detection
)
{
    //constexpr double initial_sor_std_dev_thr = 1.1;


    // TODO : edit the algorithm so that only one variable is created

    pcl::PointCloud<pcl::PointXYZ>::Ptr work_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cones_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_of_mass(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*src_cloud, *work_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(work_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(cut_bottom, cut_top);
    pass.filter(*work_cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(work_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(initial_sor_std_dev_thr);
    sor.filter(*work_cloud);


    std::vector<pcl::PointIndices> eucl_cluster_indxes;
    removeWalls(work_cloud, eucl_cluster_indxes);

    //TODO: Export these controls
    auto cone_clusters = detectCones(
        work_cloud,
        eucl_cluster_indxes,
        ideal_cone_model,
        40,
        /*0.005*/ detect_cones_fitness_detection,
        -0.7,
        0.2,
        0.5,
        13.0,
        visualize_cone_detection,
        verbose
    );

    // TODO handle case when no clusters are found.


    for (int cone_idx: cone_clusters)
    {
        std::cout << "Cone " << cone_idx << " detected\n";
        for (const auto &idx: eucl_cluster_indxes[cone_idx].indices)
        {
            std::cout << "Added point " << (*work_cloud)[idx] << " to cloud\n";
            cones_cloud->push_back((*work_cloud)[idx]);
        }

        cones_cloud->width = cones_cloud->size();
        cones_cloud->height = 1;
        cones_cloud->is_dense = true;
    }



    for (int cone_idx: cone_clusters)
    {
        float baric_x = 0;
        float baric_y = 0;
        float baric_z = 0;

        std::cout << "Cone " << cone_idx << " detected\n";
        for (const auto &idx: eucl_cluster_indxes[cone_idx].indices)
        {
            /*cones_cloud->push_back((*work_cloud)[idx]);*/

            std::cout << "Point in coord " << (*work_cloud)[idx].x << ", " << (*work_cloud)[idx].y << ", " << (*work_cloud)
                    [idx].z << "\n";
            baric_x += (*work_cloud)[idx].x;
            baric_y += (*work_cloud)[idx].y;
            baric_z += (*work_cloud)[idx].z;
        }

        baric_x = baric_x / eucl_cluster_indxes[cone_idx].indices.size();
        baric_y = baric_y / eucl_cluster_indxes[cone_idx].indices.size();
        baric_z = baric_z / eucl_cluster_indxes[cone_idx].indices.size();

        pcl::PointXYZ center_pt(baric_x, baric_y, baric_z);
        centers_of_mass->push_back(center_pt);
    }

    ConeDetectionResults results;
    results.centers_of_mass = centers_of_mass;
    results.cone_cloud = cones_cloud;

    return results;
}
