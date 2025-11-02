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


#include <iostream>
#include <thread>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main00 ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_for_stat_noise (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered_for_stat_noise);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered_for_stat_noise << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered_for_stat_noise, false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered_for_stat_noise);
    writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered_for_stat_noise, false);

    return (0);
}







struct color {
    int red;
    int green;
    int blue;
};



int safe_index(const int n, const int max) {
    return (n - 1) % max + 1;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
boh()
{
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_not_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ee(new pcl::PointCloud<pcl::PointXYZ>);

    // Rotation
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));


    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read("cones.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." <<
            std::endl;

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_not_filtered);




    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." <<
            std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    color colors[3];

    colors[0] = {255, 0, 0};
    colors[1] = {0, 255, 0};
    colors[2] = {255, 0, 255};


    int nr_iterations = 0;
    int nr_points = (int) cloud_filtered->size();

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::ExtractIndices<pcl::PointXYZ> extract_not_surfaces;

    while (cloud_filtered->size() > 0.4 * nr_points)
    {

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }



        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height <<
                " data points." << std::endl;

        int colors_index;
        colors_index = safe_index(nr_iterations, sizeof(colors));
        colors_index = 0;

        pcl::transformPointCloud(*cloud_p, *cloud_p, transform_2);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
            cloud_p,
            colors[colors_index].red,
            colors[colors_index].green,
            colors[colors_index].blue);

        //viewer.addPointCloud(cloud_p, handler, std::to_string(nr_iterations));



        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << nr_iterations << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);

        // Create the filtering object
        extract.setInputCloud(cloud_not_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_ee);
        cloud_not_filtered.swap(cloud_ee);


        nr_iterations++;
    }

    int colors_index;
    colors_index = safe_index(nr_iterations, sizeof(colors));
    colors_index = 2;

    pcl::transformPointCloud(*cloud_filtered, *cloud_p, transform_2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        cloud_p,
        colors[colors_index].red,
        colors[colors_index].green,
        colors[colors_index].blue);

    viewer.addPointCloud(cloud_p, handler, std::to_string(nr_iterations));




    colors_index = 1;

    pcl::transformPointCloud(*cloud_not_filtered, *cloud_not_filtered, transform_2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler2(
            cloud_not_filtered,
            colors[colors_index].red,
            colors[colors_index].green,
            colors[colors_index].blue);

    //viewer.addPointCloud(cloud_not_filtered, handler2, std::to_string(nr_iterations));


    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPosition(0, 0);


    /*while (!viewer.wasStopped())
    {
        viewer.spinOnce(); // Display the visualizer until the 'q' key is pressed
    }

    return(0);*/

    return cloud_p;

}



int
main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ee(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_be_rem(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, 0.0;
    //transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));
    //transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boh();

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
    reg.setMinClusterSize(500);
    reg.setMaxClusterSize(1000000);
    //reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(300);
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);




    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int cluster_idx;
    cluster_idx = 8;
    extract.setInputCloud(cloud);
    pcl::PointIndices::Ptr cluster_ptr2(new pcl::PointIndices(clusters[cluster_idx]));
    extract.setIndices (cluster_ptr2);
    extract.setKeepOrganized(true);
    extract.filter(*cloud_one_cluster);

    pcl::PointCloud<pcl::PointXYZ>::Ptr desired_object_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::copyPointCloud(*cloud, *desired_object_cloud);




    /*
    extract.setInputCloud(desired_object_cloud);
    for (int i = 0; i < clusters.size(); i++)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(clusters[i]));
        extract.setIndices (cluster_ptr);
        extract.setNegative(true);
        extract.setKeepOrganized(true);
        extract.filter(*desired_object_cloud);
    }*/



    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (desired_object_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter(*desired_object_cloud);



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::PCLVisualizer viewer("Cluster viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
            desired_object_cloud, 255,255,255);

    // Use PCLVisualizer instead of CloudViewer
    pcl::transformPointCloud(*desired_object_cloud, *desired_object_cloud, transform);
    pcl::transformPointCloud(*cloud, *cloud, transform);

    //viewer.addPointCloud(colored_cloud, "segcloud");

    viewer.addPointCloud(desired_object_cloud, "cloud");

    //viewer.addPointCloud(cloud_one_cluster, "cloud");


    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);


    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return (0);
}