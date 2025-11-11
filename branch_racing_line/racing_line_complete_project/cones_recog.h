#ifndef CONES_RECOG_H
#define CONES_RECOG_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <string>
#include <vector>

namespace cones {

// Procedural functions with many shared parameters
// (This will be refactored into a class)

/**
 * Load and preprocess point cloud from file
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndPreprocess(
    const std::string& filename,
    double voxel_leaf_size,
    int sor_mean_k,
    double sor_stddev_mul_thresh,
    double plane_distance_threshold,
    double min_points_ratio);

/**
 * Apply coordinate transformation
 */
void applyTransformation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double rotation_x_deg,
    double rotation_z_deg);

/**
 * Cluster point cloud into individual cones
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterCones(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size);

/**
 * Extract cone centers from clusters
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr extractConeCenters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

/**
 * Match detected cones with model using ICP
 */
bool matchWithModel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& detected_cone,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& model_cone,
    double icp_max_correspondence_distance,
    int icp_max_iterations,
    double icp_transformation_epsilon,
    double icp_fitness_threshold,
    Eigen::Matrix4f& transformation);

/**
 * Filter cones by distance from a reference point
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr filterByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cones,
    double ref_x,
    double ref_y,
    double max_distance);

/**
 * Main recognition pipeline - demonstrates parameter explosion
 */
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
    double filter_max_distance);

} // namespace cones

#endif // CONES_RECOG_H
