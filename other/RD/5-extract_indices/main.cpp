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
#include <pcl/visualization/pcl_visualizer.h>

#include "ColorUtilities.h"


int
main()
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

    ColorUtilities color_utilities;

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


        pcl::transformPointCloud(*cloud_p, *cloud_p, transform_2);


        int colors_index = 0;
        ColorUtilities::color color = color_utilities.getColor(colors_index);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
            cloud_p,
            color.red,
            color.green,
            color.blue);

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

    int colors_index = 2;
    ColorUtilities::color color = color_utilities.getColor(colors_index);

    pcl::transformPointCloud(*cloud_filtered, *cloud_p, transform_2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        cloud_p,
        color.red,
        color.green,
        color.blue);

    viewer.addPointCloud(cloud_p, handler, std::to_string(nr_iterations));




    colors_index = 1;
    color = color_utilities.getColor(colors_index);

    pcl::transformPointCloud(*cloud_not_filtered, *cloud_not_filtered, transform_2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler2(
            cloud_not_filtered,
            color.red,
            color.green,
            color.blue);

    //viewer.addPointCloud(cloud_not_filtered, handler2, std::to_string(nr_iterations));


    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPosition(0, 0);


    while (!viewer.wasStopped())
    {
        viewer.spinOnce(); // Display the visualizer until the 'q' key is pressed
    }

    return(0);

}
