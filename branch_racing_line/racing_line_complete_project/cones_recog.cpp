#include <iostream>
#include <thread>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>


// TODO Convert to class


void compute_min_max(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices &indices,
                     pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt)
{
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
}


void performVisualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cone_model,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cluster)
{
    pcl::visualization::PCLVisualizer viewer("Cone match visualization");
    viewer.setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_color(cone_model, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
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


/* TODO return false if fails, true if it doesn't. */
bool processClusterForConeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int icp_max_iter,
                                    double fitness_threshold, double height_threshold, double min_cone_height,
                                    double max_cone_height, float max_yaw_deg, bool visualize,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cone_model,
                                    std::vector<int> &matched_clusters,
                                    pcl::console::TicToc time, int &cluster_id, const pcl::PointIndices &indices,
                                    bool verbose)
{
    if (indices.indices.size() < 10)
    {
        cluster_id++;
        return true;
    }

    // --- Compute min and max Y (height) of the cluster ---
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    compute_min_max(cloud, indices, min_pt, max_pt);

    // --- Height filtering: discard cluster if too high above ground --
    if (min_pt.y > height_threshold)
    {
        std::cout << "Cluster " << cluster_id
                << " ignored (lowest point y=" << min_pt.y
                << " > " << height_threshold << ")\n";
        cluster_id++;
        return true;
    }

    // --- Height extent filtering: discard clusters that are too short ---
    double cluster_height = max_pt.y - min_pt.y;

    if (cluster_height < min_cone_height)
    {
        std::cout << "Cluster " << cluster_id
                << " ignored (height = " << cluster_height
                << " < " << min_cone_height << ")\n";
        cluster_id++;
        return true;
    }

    if (cluster_height > max_cone_height)
    {
        std::cout << "Cluster " << cluster_id
                << " ignored (height = " << cluster_height
                << " < " << min_cone_height << ")\n";
        cluster_id++;
        return true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cluster_cloud);
    icp.setInputTarget(cone_model);
    icp.setMaximumIterations(icp_max_iter);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-8);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cluster(new pcl::PointCloud<pcl::PointXYZ>);
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
        const float yaw = std::atan2(R(0, 2), R(2, 2)); // standard yaw from rotation matrix

        // Limit rotation around Y
        const float yaw_deg = std::abs(yaw * 180.0f / M_PI);

        if (verbose)
            std::cout << "Converged | Fitness = " << score
                    << " | Yaw = " << yaw_deg << "°" << std::endl;

        if (score < fitness_threshold && yaw_deg < max_yaw_deg)
        {
            matched_clusters.push_back(cluster_id);
            if (verbose) std::cout << "  ✅ Cluster " << cluster_id << " matches the cone model!\n";

            if (visualize) performVisualization(cone_model, aligned_cluster);
        } else if (yaw_deg >= max_yaw_deg)
        {
            if (verbose)
                std::cout << "  ❌ Rejected: rotation around Y (" << yaw_deg
                        << "°) exceeds " << max_yaw_deg << "° threshold.\n";
        }
    } else
    {
        std::cout << "ICP failed to converge.\n";
    }
    return false;
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
 * @param max_yaw_deg Threshold of rotation on the y axis: is a cluster is too rotated, it is then rej
 * @param visualize Whether to visualize matching clusters
 * @param verbose Console logs are verbose
 * @return std::vector<int> Indices of clusters that matched the cone
 */
std::vector<int> detectCones(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<pcl::PointIndices> &cluster_indices,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cone_model,
    int icp_max_iter = 40,
    double fitness_threshold = 0.01,
    double height_threshold = -0.01, // <-- adjust as needed (meters)
    double min_cone_height = 0.3, // <-- minimum expected cone height in meters
    double max_cone_height = 0.5,
    float max_yaw_deg = 20.0f,
    bool visualize = true,
    bool verbose = false)
{
    // TODO da spostare in main parte che legge il file

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
        if (processClusterForConeDetection(cloud, icp_max_iter, fitness_threshold, height_threshold, min_cone_height,
                                           max_cone_height, max_yaw_deg, visualize, cone_model, matched_clusters, time,
                                           cluster_id,
                                           indices, verbose)
        )
        {
            continue;
        }

        cluster_id++;
    }

    std::cout << "\nDetected " << matched_clusters.size() << " cone-like clusters.\n";
    return matched_clusters;
}
