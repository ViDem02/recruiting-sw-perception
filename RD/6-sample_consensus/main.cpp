/* \author Bastian Steder */

#include <iostream>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

typedef pcl::PointXYZ PointType;


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
main00()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_for_stat_noise(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered_for_stat_noise);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered_for_stat_noise << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered_for_stat_noise, false);

    sor.setNegative(true);
    sor.filter(*cloud_filtered_for_stat_noise);
    writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered_for_stat_noise, false);

    return (0);
}


struct color
{
    int red;
    int green;
    int blue;
};


int safe_index(const int n, const int max)
{
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
    transform_2.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));


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


void
main_experiment()
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
    extract.setIndices(cluster_ptr2);
    extract.setKeepOrganized(true);
    extract.filter(*cloud_one_cluster);

    pcl::PointCloud<pcl::PointXYZ> desired_object_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr desired_object_cloud_ptr(&desired_object_cloud);
    pcl::copyPointCloud(*cloud, *desired_object_cloud_ptr);


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
    sor.setInputCloud(desired_object_cloud_ptr);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*desired_object_cloud_ptr);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::visualization::PCLVisualizer viewer("Cluster viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        desired_object_cloud_ptr, 255, 255, 255);

    // Use PCLVisualizer instead of CloudViewer
    pcl::transformPointCloud(*desired_object_cloud_ptr, *desired_object_cloud_ptr, transform);
    pcl::transformPointCloud(*cloud, *cloud, transform);

    //viewer.addPointCloud(colored_cloud, "segcloud");

    viewer.addPointCloud(desired_object_cloud_ptr, "cloud");

    //viewer.addPointCloud(cloud_one_cluster, "cloud");


    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);

    /*
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return (0);*/

    pcl::io::savePCDFileASCII("desire.pcd", desired_object_cloud);
}


// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

// --------------
// -----Help-----
// --------------
void
printUsage(const char *progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
            << "-c <int>     coordinate frame (default " << (int) coordinate_frame << ")\n"
            << "-m           Treat all unseen points as maximum range readings\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
            << "default " << support_size << ")\n"
            << "-h           this help\n"
            << "\n\n";
}

//void
//setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
//{
//Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
//Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
//Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
//viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
//look_at_vector[0], look_at_vector[1], look_at_vector[2],
//up_vector[0], up_vector[1], up_vector[2]);
//}

// --------------
// -----Main-----
// --------------
int
main(int argc, char **argv)
{

    main_experiment();
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int) coordinate_frame << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
        std::cout << "Setting support size to " << support_size << ".\n";
    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
        std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
    angular_resolution = pcl::deg2rad(angular_resolution);

    // ------------------------------------------------------------------
    // -----Read pcd file or create example point cloud if not given-----
    // ------------------------------------------------------------------
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
    if (/*!pcd_filename_indices.empty()*/ true)
    {
        std::string filename = /*argv[pcd_filename_indices[0]];*/ "desire.pcd";
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
        {
            std::cerr << "Was not able to open file \"" << filename << "\".\n";
            printUsage(argv[0]);
            return 0;
        }
        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                                 point_cloud.sensor_origin_[1],
                                                                 point_cloud.sensor_origin_[2])) *
                            Eigen::Affine3f(point_cloud.sensor_orientation_);
        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
            std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
    } else
    {
        setUnseenToMaxRange = true;
        std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f)
        {
            for (float y = -0.5f; y <= 0.5f; y += 0.01f)
            {
                PointType point;
                point.x = x;
                point.y = y;
                point.z = 2.0f - y;
                point_cloud.push_back(point);
            }
        }
        point_cloud.width = point_cloud.size();
        point_cloud.height = 1;
    }

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image = *range_image_ptr;
    range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(
        range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
    //viewer.addCoordinateSystem (1.0f, "global");
    //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
    //setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    range_image_widget.showRangeImage(range_image);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;
    //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);
    std::cout << "Found " << keypoint_indices.size() << " key points.\n";

    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    //for (std::size_t i=0; i<keypoint_indices.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices[i]%range_image.width,
    //keypoint_indices[i]/range_image.width);

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints = *keypoints_ptr;
    keypoints.resize(keypoint_indices.size());
    for (std::size_t i = 0; i < keypoint_indices.size(); ++i)
        keypoints[i].getVector3fMap() = range_image[keypoint_indices[i]].getVector3fMap();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped())
    {
        range_image_widget.spinOnce(); // process GUI events
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}
