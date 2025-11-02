#include <iostream>
#include <thread>
#include <vector>


#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill

#include "ColorUtilities.h"


Eigen::Affine3f
get_rotation_matrix()
{
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
    return transform_2;
}

int
main ()
{

    const std::string file_name = "cones.pcd";
    //const std::string file_name = "Statues_4.pcd";

    pcl::visualization::PCLVisualizer viewer("VISUAL");
    int j = 0;
    pcl::PCDWriter writer;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PCDReader reader;

    reader.read<pcl::PointXYZ> (file_name, *cloud);
    reader.read<pcl::PointXYZ> (file_name, *original_cloud);

    pcl::transformPointCloud(*cloud, *cloud, get_rotation_matrix());
    pcl::transformPointCloud(*original_cloud, *original_cloud, get_rotation_matrix());


    /*
    // Statistical Outlier remover
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);*/


    // Downsample
/*
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloud);
    vox_grid.setLeafSize(0.01f, 0.01f, 0.01f);
    vox_grid.filter(*cloud);
*/

   //cutting high points
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-10.0, 0.6);
    //pass.setNegative (true);
    pass.filter (*cloud);


    /*

    // Segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> seg_extract;

    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    const int initial_nr_points = cloud->size();

    while (true)
    {
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        seg_extract.setInputCloud(cloud);
        seg_extract.setIndices(inliers);
        seg_extract.setNegative(true);
        seg_extract.setKeepOrganized(false);
        seg_extract.filter(*cloud);

        if (cloud->size() < 0.4 * initial_nr_points) break;
    }*/




    // Statistical Outlier remover
    /*
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (2);
    sor.filter (*cloud);*/







    //Region growing
    pcl::ExtractIndices<pcl::PointXYZ> reg_grow_extract;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setInputCloud(cloud);
    reg.setMinClusterSize(500);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(300);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold( 1/ 180.0 * M_PI);
    reg.setCurvatureThreshold(1);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    constexpr int clusters_to_be_removed [] = {1,2,3};

    reg_grow_extract.setInputCloud(cloud);

    j = 0;
    for (const auto &cluster: clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx: cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZ>("reg_cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false);
        j++;
    }



    reg_grow_extract.setInputCloud(cloud);
    //for (const auto cluster_index : clusters_to_be_removed)
    for (int cluster_index = 0; cluster_index < clusters.size(); cluster_index++)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(clusters[cluster_index]));
        reg_grow_extract.setIndices (cluster_ptr);
        reg_grow_extract.setNegative(true);
        reg_grow_extract.setKeepOrganized(true);
        reg_grow_extract.filter(*cloud);
    }





    //Conditional Euclidian clustering
    // Creating the KdTree object for the search method of the extraction


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> eucl_cluster_extr;
    eucl_cluster_extr.setClusterTolerance(0.1);
    eucl_cluster_extr.setMinClusterSize(15);
    eucl_cluster_extr.setMaxClusterSize(500);
    eucl_cluster_extr.setSearchMethod(tree2);
    eucl_cluster_extr.setInputCloud(cloud);
    eucl_cluster_extr.extract(cluster_indices);

    j = 0;
    for (const auto &cluster: cluster_indices)
    {
        ColorUtilities color_util;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &idx: cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        auto [rr, gg, bb] = color_util.getColor(j);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
            cloud_cluster,
            rr,
            gg,
            0);

        viewer.addPointCloud(cloud_cluster, handler, std::to_string(j) );
        j++;
    }

    j = 0;
    for (const auto &cluster: cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx: cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "cone cluster " << j <<  ": " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false);
        j++;
    }


    // Visualizer
    //viewer.addPointCloud(cloud, "Result");
    //viewer.addPointCloud(original_cloud, "original");
    //viewer.addPointCloud(colored_cloud, "Colors");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }



}