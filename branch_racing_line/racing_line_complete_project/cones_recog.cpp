#include "cones_recog.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

namespace cones {

pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndPreprocess(
    const std::string& filename,
    double voxel_leaf_size,
    int sor_mean_k,
    double sor_stddev_mul_thresh,
    double plane_distance_threshold,
    double min_points_ratio)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return cloud;
    }
    
    // Voxel grid downsampling
    if (voxel_leaf_size > 0) {
        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        vox_grid.setInputCloud(cloud);
        vox_grid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        vox_grid.filter(*cloud);
    }
    
    // Statistical outlier removal
    if (sor_mean_k > 0) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_mean_k);
        sor.setStddevMulThresh(sor_stddev_mul_thresh);
        sor.filter(*cloud);
    }
    
    // Ground plane removal
    const int initial_nr_points = cloud->size();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> seg_extract;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_distance_threshold);
    
    while (cloud->size() > min_points_ratio * initial_nr_points) {
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() == 0) break;
        
        seg_extract.setInputCloud(cloud);
        seg_extract.setIndices(inliers);
        seg_extract.setNegative(true);
        seg_extract.filter(*cloud);
    }
    
    return cloud;
}

void applyTransformation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double rotation_x_deg,
    double rotation_z_deg)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(rotation_x_deg * M_PI / 180.0, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(rotation_z_deg * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterCones(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& idx : indices.indices) {
            cluster->push_back(cloud->points[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }
    
    return clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extractConeCenters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>());
    
    for (const auto& cluster : clusters) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        
        pcl::PointXYZ center;
        center.x = centroid[0];
        center.y = centroid[1];
        center.z = centroid[2];
        centers->push_back(center);
    }
    
    return centers;
}

bool matchWithModel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& detected_cone,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& model_cone,
    double icp_max_correspondence_distance,
    int icp_max_iterations,
    double icp_transformation_epsilon,
    double icp_fitness_threshold,
    Eigen::Matrix4f& transformation)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(detected_cone);
    icp.setInputTarget(model_cone);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
    icp.setMaximumIterations(icp_max_iterations);
    icp.setTransformationEpsilon(icp_transformation_epsilon);
    
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);
    
    if (icp.hasConverged() && icp.getFitnessScore() < icp_fitness_threshold) {
        transformation = icp.getFinalTransformation();
        return true;
    }
    
    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cones,
    double ref_x,
    double ref_y,
    double max_distance)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    
    double max_dist_sq = max_distance * max_distance;
    
    for (const auto& cone : cones->points) {
        double dx = cone.x - ref_x;
        double dy = cone.y - ref_y;
        double dist_sq = dx*dx + dy*dy;
        
        if (dist_sq <= max_dist_sq) {
            filtered->push_back(cone);
        }
    }
    
    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr recognizeCones(
    const std::string& scan_file,
    const std::string& model_file,
    double voxel_leaf_size,
    int sor_mean_k,
    double sor_stddev_mul_thresh,
    double plane_distance_threshold,
    double min_points_ratio,
    double rotation_x_deg,
    double rotation_z_deg,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    double icp_max_correspondence_distance,
    int icp_max_iterations,
    double icp_transformation_epsilon,
    double icp_fitness_threshold,
    double filter_ref_x,
    double filter_ref_y,
    double filter_max_distance)
{
    // Load and preprocess
    auto cloud = loadAndPreprocess(
        scan_file, voxel_leaf_size, sor_mean_k, sor_stddev_mul_thresh,
        plane_distance_threshold, min_points_ratio);
    
    // Apply transformation
    applyTransformation(cloud, rotation_x_deg, rotation_z_deg);
    
    // Cluster into individual cones
    auto clusters = clusterCones(
        cloud, cluster_tolerance, min_cluster_size, max_cluster_size);
    
    // Extract centers
    auto centers = extractConeCenters(clusters);
    
    // Filter by distance
    auto filtered_centers = filterByDistance(
        centers, filter_ref_x, filter_ref_y, filter_max_distance);
    
    return filtered_centers;
}

} // namespace cones
