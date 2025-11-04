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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <iostream>

// Type alias
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <pcl/common/common.h>   // <-- required for getMinMax3D

#include <iostream>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <iostream>
#include <vector>
#include <limits>

#include "ColorUtilities.h"


bool
customDensityCondition(const PointT &pt_a, const PointT &pt_b, float squaredDistance)
{
    // per esempio: permetti aggregazione solo se la distanza è < tol e densità locale > soglia
    return (squaredDistance < (0.05f * 0.05f)); // esempio: distanza < 5 cm
}

/**
 * @brief Iteratively decide which clusters correspond to cones.
 *
 * @param cloud Input point cloud from which clusters were extracted.
 * @param cluster_indices Vector of clusters (each containing point indices).
 * @param min_height Minimum expected cone height (e.g. 0.3 m).
 * @param max_height Maximum expected cone height (e.g. 0.9 m).
 * @param min_radius Minimum base radius (e.g. 0.1 m).
 * @param max_radius Maximum base radius (e.g. 0.4 m).
 * @param min_points Minimum number of points in a valid cone cluster.
 * @param max_points Maximum number of points in a valid cone cluster.
 * @return std::vector<int> Indices of clusters likely to be cones.
 */
std::vector<int> detectConesIteratively(
    const PointCloudT::Ptr &cloud,
    const std::vector<pcl::PointIndices> &cluster_indices,
    float min_height = 0.3f,
    float max_height = 0.5f,
    float min_radius = 0.10f,
    float max_radius = 0.20f,
    int min_points = 10,
    int max_points = 2000)
{
    std::vector<int> cone_clusters;

    int cluster_id = 0;
    for (const auto &indices: cluster_indices)
    {
        // Skip clusters that are too small or too large
        if ((int) indices.indices.size() < min_points ||
            (int) indices.indices.size() > max_points)
        {
            cluster_id++;
            continue;
        }


        //min max computation
        pcl::PointXYZ min_pt, max_pt;
        bool first = true;

        for (int idx: indices.indices)
        {
            const auto &p = cloud->points[idx];
            if (first)
            {
                min_pt = max_pt = p;
                first = false;
            } else
            {
                if (p.x < min_pt.x) min_pt.x = p.x;
                if (p.y < min_pt.y) min_pt.y = p.y;
                if (p.z < min_pt.z) min_pt.z = p.z;

                if (p.x > max_pt.x) max_pt.x = p.x;
                if (p.y > max_pt.y) max_pt.y = p.y;
                if (p.z > max_pt.z) max_pt.z = p.z;
            }
        }

        float height = max_pt.y - min_pt.y;
        float width_x = max_pt.x - min_pt.x;
        float width_z = max_pt.z - min_pt.z;
        float base_radius = std::max(width_x, width_z) / 2.0f;

        // Simple geometric heuristics for a traffic cone
        bool height_ok = (height >= min_height && height <= max_height);
        bool radius_ok = (base_radius >= min_radius && base_radius <= max_radius);
        bool slender_shape = (height / (2 * base_radius) > 1.5f); // roughly cone-like

        if (height_ok && radius_ok && slender_shape)
        {
            cone_clusters.push_back(cluster_id);
        }

        cluster_id++;
    }

    return cone_clusters;
}


/**
 * @brief Detect clusters that match a cone model using ICP.
 *        If a match is found, visualize it with PCLVisualizer.
 *
 * @param cloud Input LiDAR point cloud (the full scan)
 * @param cluster_indices Clusters from segmentation
 * @param cone_model_ply Path to cone model .ply file
 * @param icp_max_iter Maximum ICP iterations
 * @param fitness_threshold ICP fitness score threshold for match
 * @param height_threshold Clusters with lowest point higher than this height will be disregarded
 * @param min_cone_height  Minimum expected cone height in meters
 * @param max_cone_height Maximum expected cone height in meters
 * @param visualize Whether to visualize matching clusters
 * @return std::vector<int> Indices of clusters that matched the cone
 */
std::vector<int> detectConesUsingICPandVisualize(
    const PointCloudT::Ptr &cloud,
    const std::vector<pcl::PointIndices> &cluster_indices,
    const std::string &cone_model_ply,
    int icp_max_iter = 40,
    double fitness_threshold = 0.01,
    double height_threshold = -0.7, // <-- adjust as needed (meters)
    double min_cone_height = 0.3, // <-- minimum expected cone height in meters
    double max_cone_height = 0.5,
    float max_yaw_deg = 20.0f,
    bool visualize = true)
{
    PointCloudT::Ptr cone_model(new PointCloudT);
    if (pcl::io::loadPLYFile<PointT>(cone_model_ply, *cone_model) == -1)
    {
        PCL_ERROR("Couldn't read the cone model PLY file.\n");
        return {};
    }

    std::cout << "Loaded reference cone model with " << cone_model->size() << " points\n";

    // Center the model around its centroid
    Eigen::Vector4f model_centroid;
    pcl::compute3DCentroid(*cone_model, model_centroid);
    Eigen::Matrix4f T_center_model = Eigen::Matrix4f::Identity();
    T_center_model(0, 3) = -model_centroid[0];
    T_center_model(1, 3) = -model_centroid[1];
    T_center_model(2, 3) = -model_centroid[2];
    pcl::transformPointCloud(*cone_model, *cone_model, T_center_model);

    std::vector<int> matched_clusters;
    pcl::console::TicToc time;

    int cluster_id = 0;
    for (const auto &indices: cluster_indices)
    {
        if (indices.indices.size() < 10)
        {
            cluster_id++;
            continue;
        }

        // --- Compute min and max Y (height) of the cluster ---
        pcl::PointXYZ min_pt, max_pt;
        bool first = true;

        for (int idx: indices.indices)
        {
            const auto &p = cloud->points[idx];
            if (first)
            {
                min_pt = max_pt = p;
                first = false;
            } else
            {
                if (p.x < min_pt.x) min_pt.x = p.x;
                if (p.y < min_pt.y) min_pt.y = p.y;
                if (p.z < min_pt.z) min_pt.z = p.z;

                if (p.x > max_pt.x) max_pt.x = p.x;
                if (p.y > max_pt.y) max_pt.y = p.y;
                if (p.z > max_pt.z) max_pt.z = p.z;
            }
        }

        // --- Height filtering: discard cluster if too high above ground --
        if (min_pt.y > height_threshold)
        {
            std::cout << "Cluster " << cluster_id
                    << " ignored (lowest point y=" << min_pt.y
                    << " > " << height_threshold << ")\n";
            cluster_id++;
            continue;
        }

        // --- Height extent filtering: discard clusters that are too short ---
        double cluster_height = max_pt.y - min_pt.y;

        if (cluster_height < min_cone_height)
        {
            std::cout << "Cluster " << cluster_id
                    << " ignored (height = " << cluster_height
                    << " < " << min_cone_height << ")\n";
            cluster_id++;
            continue;
        }

        if (cluster_height > max_cone_height)
        {
            std::cout << "Cluster " << cluster_id
                    << " ignored (height = " << cluster_height
                    << " < " << min_cone_height << ")\n";
            cluster_id++;
            continue;
        }

        PointCloudT::Ptr cluster_cloud(new PointCloudT);
        pcl::copyPointCloud(*cloud, indices.indices, *cluster_cloud);

        // Center the cluster
        Eigen::Vector4f cluster_centroid;
        pcl::compute3DCentroid(*cluster_cloud, cluster_centroid);
        Eigen::Matrix4f T_center_cluster = Eigen::Matrix4f::Identity();
        T_center_cluster(0, 3) = -cluster_centroid[0];
        T_center_cluster(1, 3) = -cluster_centroid[1];
        T_center_cluster(2, 3) = -cluster_centroid[2];
        pcl::transformPointCloud(*cluster_cloud, *cluster_cloud, T_center_cluster);

        // --- ICP alignment ---
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(cluster_cloud);
        icp.setInputTarget(cone_model);
        icp.setMaximumIterations(icp_max_iter);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-8);

        PointCloudT::Ptr aligned_cluster(new PointCloudT);
        time.tic();
        icp.align(*aligned_cluster);
        double t_ms = time.toc();

        std::cout << "Cluster " << cluster_id << " | ";
        if (icp.hasConverged())
        {
            double score = icp.getFitnessScore();
            Eigen::Matrix4f transform = icp.getFinalTransformation();

            // --- Extract rotation around Y axis (yaw) ---
            // Rotation matrix part
            Eigen::Matrix3f R = transform.block<3, 3>(0, 0);
            // Compute yaw (rotation around Y)
            float yaw = std::atan2(R(0, 2), R(2, 2)); // standard yaw from rotation matrix

            // Limit rotation around Y
            float yaw_deg = std::abs(yaw * 180.0f / M_PI);

            std::cout << "Converged | Fitness = " << score
                    << " | Yaw = " << yaw_deg << "°" << std::endl;

            if (score < fitness_threshold && yaw_deg < max_yaw_deg)
            {
                matched_clusters.push_back(cluster_id);
                std::cout << "  ✅ Cluster " << cluster_id << " matches the cone model!\n";

                if (visualize)
                {
                    pcl::visualization::PCLVisualizer viewer("Cone match visualization");
                    viewer.setBackgroundColor(0, 0, 0);

                    pcl::visualization::PointCloudColorHandlerCustom<PointT> model_color(cone_model, 255, 255, 255);
                    pcl::visualization::PointCloudColorHandlerCustom<PointT>
                            cluster_color(aligned_cluster, 255, 50, 50);

                    viewer.addPointCloud(cone_model, model_color, "model");
                    viewer.addPointCloud(aligned_cluster, cluster_color, "cluster");

                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model");
                    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                            "cluster");

                    viewer.addText("White: Cone model\nRed: Cluster", 10, 10, "info");
                    viewer.setSize(1000, 800);
                    while (!viewer.wasStopped())
                    {
                        viewer.spinOnce();
                    }
                }
            } else if (yaw_deg >= max_yaw_deg)
            {
                std::cout << "  ❌ Rejected: rotation around Y (" << yaw_deg
                        << "°) exceeds " << max_yaw_deg << "° threshold.\n";
            }
        } else
        {
            std::cout << "ICP failed to converge.\n";
        }

        cluster_id++;
    }

    std::cout << "\nDetected " << matched_clusters.size() << " cone-like clusters.\n";
    return matched_clusters;
}


Eigen::Affine3f
get_rotation_matrix()
{
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
    return transform_2;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr
get_cones_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const float cut_bottom = -10.0,
    const float cut_top = -0.2
    )
{
    const pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(cut_bottom, cut_top);
    pass.filter(*cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.8);
    sor.filter(*cloud);

    pcl::ExtractIndices<pcl::PointXYZ> reg_grow_extract;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();


    pcl::removeNaNFromPointCloud(*cloud, *indices);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    reg.setInputCloud(cloud);
    reg.setMinClusterSize(500);
    reg.setMaxClusterSize(10000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(300);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(1 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1);

    reg.extract(clusters);

    reg_grow_extract.setInputCloud(cloud);
    for (const auto & cluster : clusters)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(cluster));
        reg_grow_extract.setIndices(cluster_ptr);
        reg_grow_extract.setNegative(true);
        reg_grow_extract.setKeepOrganized(true);
        reg_grow_extract.filter(*cloud);
    }


    return result_cloud;
}






int
main()
{
    const std::string file_name = "cones.pcd";
    //const std::string file_name = "Statues_4.pcd";

    pcl::visualization::PCLVisualizer viewer("VISUAL");
    int j = 0;
    pcl::PCDWriter writer;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PCDReader reader;

    reader.read<pcl::PointXYZ>(file_name, *cloud);
    reader.read<pcl::PointXYZ>(file_name, *original_cloud);

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
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10.0, -0.2);
    //pass.setNegative (true);
    pass.filter(*cloud);


    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.8);
    sor.filter(*cloud);

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
    reg.setSmoothnessThreshold(1 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

    constexpr int clusters_to_be_removed[] = {1, 2, 3};

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
    for (const auto & cluster : clusters)
    {
        pcl::PointIndices::Ptr cluster_ptr(new pcl::PointIndices(cluster));
        reg_grow_extract.setIndices(cluster_ptr);
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
    eucl_cluster_extr.setClusterTolerance(0.05);
    eucl_cluster_extr.setMinClusterSize(8);
    eucl_cluster_extr.setMaxClusterSize(500);
    eucl_cluster_extr.setSearchMethod(tree2);
    eucl_cluster_extr.setInputCloud(cloud);
    eucl_cluster_extr.extract(cluster_indices);


    /*
        pcl::ConditionalEuclideanClustering<PointT> cec(true);
        cec.setInputCloud(cloud);
        cec.setConditionFunction(&customDensityCondition);
        cec.setClusterTolerance(0.05);
        cec.setMinClusterSize(8);
        cec.setMaxClusterSize(500);
        std::vector<pcl::PointIndices> cluster_indices;
        cec.segment(clusters);
    */


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

        /*
        std::cout << "cone cluster " << j <<  ": " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false);
        */
        j++;
    }


    // min cone height = 0.3 ==> ottimo riconoscimento
    // dei coni, però la curva non viene vista

    // min cone height = 0.1 ==> ottimo riconoscimento
    // dei coni, però la curva non viene vista


    std::string cone_model = "cone_surface_only.ply";
    auto cone_clusters = detectConesUsingICPandVisualize(
        cloud,
        cluster_indices,
        cone_model,
        40,
        0.01,
        -0.7,
        0.2,
        0.5,
        20,
        true
    );

    for (int idx: cone_clusters)
        std::cout << "Cluster " << idx << " is likely a cone.\n";

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
        original_cloud,
        0,
        255,
        0);
    viewer.addPointCloud(original_cloud, handler, "original");

    j = 0;
    for (const auto cluster_index: cone_clusters)
    {
        ColorUtilities color_util;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &idx: cluster_indices[cluster_index].indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        auto [rr, gg, bb] = color_util.getColor(j);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(
            cloud_cluster,
            255,
            0,
            0);

        viewer.addPointCloud(cloud_cluster, handler, std::to_string(j));
        j++;
    }

    // Visualizer
    //viewer.addPointCloud(cloud, "Result");

    //viewer.addPointCloud(colored_cloud, "Colors");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
