#include "ConesRecognizer.h"
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
#include <pcl/registration/icp.h>

namespace cones {

ConesRecognizer::ConesRecognizer()
    : cone_centers_(new pcl::PointCloud<pcl::PointXYZ>()),
      cone_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
      model_cone_(nullptr)
{
}

ConesRecognizer::ConesRecognizer(const RecognizerConfig& config)
    : config_(config),
      cone_centers_(new pcl::PointCloud<pcl::PointXYZ>()),
      cone_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
      model_cone_(nullptr)
{
}

void ConesRecognizer::setConfig(const RecognizerConfig& config)
{
    config_ = config;
}

const RecognizerConfig& ConesRecognizer::getConfig() const
{
    return config_;
}

bool ConesRecognizer::loadModel(const std::string& model_file)
{
    model_cone_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    if (model_file.substr(model_file.length() - 4) == ".ply") {
        return pcl::io::loadPLYFile<pcl::PointXYZ>(model_file, *model_cone_) != -1;
    } else {
        return pcl::io::loadPCDFile<pcl::PointXYZ>(model_file, *model_cone_) != -1;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConesRecognizer::detectCones(const std::string& scan_file)
{
    // Load and preprocess
    auto cloud = loadAndPreprocess(scan_file);
    
    if (!cloud || cloud->empty()) {
        return cone_centers_;
    }
    
    // Apply transformation
    applyTransformation(cloud);
    
    // Cluster into individual cones
    clusterCones(cloud);
    
    // Extract centers
    extractConeCenters();
    
    // Filter by distance
    filterByDistance();
    
    return cone_centers_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConesRecognizer::detectCones(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (!cloud || cloud->empty()) {
        return cone_centers_;
    }
    
    // Create a copy to avoid modifying input
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    
    // Apply transformation
    applyTransformation(cloud_copy);
    
    // Cluster into individual cones
    clusterCones(cloud_copy);
    
    // Extract centers
    extractConeCenters();
    
    // Filter by distance
    filterByDistance();
    
    return cone_centers_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConesRecognizer::getCenters() const
{
    return cone_centers_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConesRecognizer::getConeCloud() const
{
    return cone_cloud_;
}

const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& ConesRecognizer::getClusters() const
{
    return clusters_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ConesRecognizer::processFrame(const std::string& scan_file)
{
    return detectCones(scan_file);
}

// Private helper methods

pcl::PointCloud<pcl::PointXYZ>::Ptr ConesRecognizer::loadAndPreprocess(const std::string& filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
        return cloud;
    }
    
    // Voxel grid downsampling
    if (config_.voxel_leaf_size > 0) {
        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        vox_grid.setInputCloud(cloud);
        vox_grid.setLeafSize(config_.voxel_leaf_size, config_.voxel_leaf_size, config_.voxel_leaf_size);
        vox_grid.filter(*cloud);
    }
    
    // Statistical outlier removal
    if (config_.sor_mean_k > 0) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(config_.sor_mean_k);
        sor.setStddevMulThresh(config_.sor_stddev_mul_thresh);
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
    seg.setDistanceThreshold(config_.plane_distance_threshold);
    
    while (cloud->size() > config_.min_points_ratio * initial_nr_points) {
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

void ConesRecognizer::applyTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(config_.rotation_x_deg * M_PI / 180.0, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(config_.rotation_z_deg * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

void ConesRecognizer::clusterCones(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    clusters_.clear();
    cone_cloud_->clear();
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(config_.cluster_tolerance);
    ec.setMinClusterSize(config_.min_cluster_size);
    ec.setMaxClusterSize(config_.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& idx : indices.indices) {
            cluster->push_back(cloud->points[idx]);
            cone_cloud_->push_back(cloud->points[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters_.push_back(cluster);
    }
}

void ConesRecognizer::extractConeCenters()
{
    cone_centers_->clear();
    
    for (const auto& cluster : clusters_) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        
        pcl::PointXYZ center;
        center.x = centroid[0];
        center.y = centroid[1];
        center.z = centroid[2];
        cone_centers_->push_back(center);
    }
}

void ConesRecognizer::filterByDistance()
{
    if (std::isinf(config_.filter_max_distance)) {
        // No filtering needed
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    double max_dist_sq = config_.filter_max_distance * config_.filter_max_distance;
    
    for (const auto& cone : cone_centers_->points) {
        double dx = cone.x - config_.filter_ref_x;
        double dy = cone.y - config_.filter_ref_y;
        double dist_sq = dx*dx + dy*dy;
        
        if (dist_sq <= max_dist_sq) {
            filtered->push_back(cone);
        }
    }
    
    cone_centers_ = filtered;
}

bool ConesRecognizer::matchWithModel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& detected_cone,
    Eigen::Matrix4f& transformation)
{
    if (!model_cone_ || model_cone_->empty()) {
        return false;
    }
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(detected_cone);
    icp.setInputTarget(model_cone_);
    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_distance);
    icp.setMaximumIterations(config_.icp_max_iterations);
    icp.setTransformationEpsilon(config_.icp_transformation_epsilon);
    
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);
    
    if (icp.hasConverged() && icp.getFitnessScore() < config_.icp_fitness_threshold) {
        transformation = icp.getFinalTransformation();
        return true;
    }
    
    return false;
}

} // namespace cones
