#include <iostream>
#include <thread>
#include <vector>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>

#include "cones_recog.h"
#include "DistinguishLeftRight.h"
#include "get_cones_cloud.h"
#include "get_obstacle_cloud.h"
#include "racingline_nlopt.h"

//TODO: To be removed
//TODO: As final step, introduce typedefs




Eigen::Affine3f
get_rotation_matrix(const float transl_on_y_axis = 0.1)
{
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, transl_on_y_axis;
    transform_2.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitY()));
    transform_2.rotate(Eigen::AngleAxisf(-M_PI / 1, Eigen::Vector3f::UnitZ()));
    return transform_2;
}


int
main()
{
    const std::string file_name = "cones.pcd";
    const std::string ideal_cone_model_name = "cone_surface_only.ply";

    constexpr double initial_sor_std_dev_thr = 1.1;
    constexpr float cone_detection_cut_bottom = -10.0;
    constexpr float cone_detection_cut_top = -0.2;
    constexpr float transl_on_y_axis = 0.1;
    constexpr float dist_left_right_max_dist = 0.5; //default 05
    constexpr double detect_cones_fitness_detection = 0.005;
    constexpr bool verbose = true;


    pcl::visualization::PCLVisualizer viewer("VISUAL");

    pcl::PCDWriter writer;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obst_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr racing_line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;

    reader.read<pcl::PointXYZ>(file_name, *original_cloud);

    pcl::transformPointCloud(*original_cloud, *original_cloud, get_rotation_matrix(
                                 transl_on_y_axis
                             ));

    pcl::copyPointCloud(*original_cloud, *cloud);

    auto [centers_of_mass, cone_cloud] = get_cones_cloud(
        cloud,
        initial_sor_std_dev_thr,
        cone_detection_cut_bottom,
        cone_detection_cut_top,
        ideal_cone_model_name, detect_cones_fitness_detection, verbose
    );

    if (! cone_cloud->empty())
    {
        auto [src_left, src_right] =
            cones::distinguishLeftRight(
                centers_of_mass,
                0,
                0,
                -45,
                +45,
                dist_left_right_max_dist);

        racing_line = get_racing_line_point_cloud(
            src_left,
            src_right);

        obst_cloud = get_obstacle_cloud(cloud, cone_cloud);
    }


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_original_cloud(
        original_cloud,
        0,
        150,
        0);
    viewer.addPointCloud(original_cloud, handler_original_cloud, "original_cloud");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_racing_line(
        racing_line,
        255,
        200,
        0);
    viewer.addPointCloud(racing_line, handler_racing_line, "racing_line");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_cones_cloud(
        cone_cloud,
        255,
        0,
        0);
    viewer.addPointCloud(cone_cloud, handler_cones_cloud, "cones_cloud");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_obst_cloud(
        obst_cloud,
        0,
        255,
        255);
    viewer.addPointCloud(obst_cloud, handler_obst_cloud, "obst_cloud");


    reference_cloud->push_back(pcl::PointXYZ(1, 0, 0));
    reference_cloud->push_back(pcl::PointXYZ(-1, 0, 0));
    reference_cloud->push_back(pcl::PointXYZ(0, 0, 1));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_reference_cloud(
        reference_cloud,
        255,
        255,
        255);
    viewer.addPointCloud(reference_cloud, handler_reference_cloud, "reference_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            15, "reference_cloud");



    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
