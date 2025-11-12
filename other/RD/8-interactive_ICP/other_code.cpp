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
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>


struct color {
    int red;
    int green;
    int blue;
};

int safe_index(const int n, const int max) {
    return (n - 1) % max + 1;
}


Eigen::Affine3f
get_rotation_matrix()
{
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
    return transform_2;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
get_cloud ()
{
    const std::string file_name = "cones.pcd";

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
    vox_grid.filter(*cloud);*/



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
    }


    // Statistical Outlier remover
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (2);
    //sor.filter (*cloud);





    /*

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
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(300);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(1.5 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();


    reg_grow_extract.setInputCloud(cloud);
    for (int i = 0; i < clusters.size(); i++)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(clusters[i]));
        reg_grow_extract.setIndices (cluster_ptr);
        reg_grow_extract.setNegative(true);
        reg_grow_extract.setKeepOrganized(true);
        reg_grow_extract.filter(*cloud);
    }
    */




    // Visualizer
    pcl::visualization::PCLVisualizer viewer("VISUAL");

    //viewer.addPointCloud(cloud, "Result");
    viewer.addPointCloud(original_cloud, "original");
    //viewer.addPointCloud(colored_cloud, "Colors");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return (cloud);

}