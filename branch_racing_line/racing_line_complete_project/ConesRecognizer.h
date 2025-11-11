#ifndef CONES_RECOGNIZER_H
#define CONES_RECOGNIZER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace cones {

/**
 * Configuration structure for cone recognition parameters
 */
struct RecognizerConfig {
    // Preprocessing parameters
    double voxel_leaf_size = 0.01;
    int sor_mean_k = 50;
    double sor_stddev_mul_thresh = 1.0;
    double plane_distance_threshold = 0.01;
    double min_points_ratio = 0.4;
    
    // Transformation parameters
    double rotation_x_deg = -90.0;
    double rotation_z_deg = 90.0;
    
    // Clustering parameters
    double cluster_tolerance = 0.05;
    int min_cluster_size = 10;
    int max_cluster_size = 10000;
    
    // ICP parameters
    double icp_max_correspondence_distance = 0.05;
    int icp_max_iterations = 50;
    double icp_transformation_epsilon = 1e-8;
    double icp_fitness_threshold = 0.05;
    
    // Filtering parameters
    double filter_ref_x = 0.0;
    double filter_ref_y = 0.0;
    double filter_max_distance = std::numeric_limits<double>::infinity();
};

/**
 * ConesRecognizer class - encapsulates cone detection and recognition logic.
 * Reduces parameter passing between functions by maintaining configuration state.
 */
class ConesRecognizer {
public:
    /**
     * Constructor with default configuration
     */
    ConesRecognizer();
    
    /**
     * Constructor with custom configuration
     */
    explicit ConesRecognizer(const RecognizerConfig& config);
    
    /**
     * Set configuration
     */
    void setConfig(const RecognizerConfig& config);
    
    /**
     * Get current configuration
     */
    const RecognizerConfig& getConfig() const;
    
    /**
     * Set model cone for ICP matching
     */
    bool loadModel(const std::string& model_file);
    
    /**
     * Detect cones from a point cloud file
     * @param scan_file Path to the PCD file
     * @return Point cloud of detected cone centers
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectCones(const std::string& scan_file);
    
    /**
     * Detect cones from an already loaded point cloud
     * @param cloud Input point cloud
     * @return Point cloud of detected cone centers
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr detectCones(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    /**
     * Get the centers of the last detected cones
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenters() const;
    
    /**
     * Get the full cone point cloud (all clusters)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getConeCloud() const;
    
    /**
     * Get individual cone clusters from the last detection
     */
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& getClusters() const;
    
    /**
     * Process a single frame (convenience method)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr processFrame(const std::string& scan_file);

private:
    // Configuration
    RecognizerConfig config_;
    
    // Model for ICP matching
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cone_;
    
    // Last detection results
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_centers_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_;
    
    // Private helper methods - operate on member state, reducing parameter passing
    
    /**
     * Load and preprocess point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndPreprocess(const std::string& filename);
    
    /**
     * Apply coordinate transformation to cloud
     */
    void applyTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    /**
     * Cluster point cloud into individual cones
     */
    void clusterCones(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    /**
     * Extract cone centers from clusters
     */
    void extractConeCenters();
    
    /**
     * Filter cones by distance from reference point
     */
    void filterByDistance();
    
    /**
     * Match detected cone with model using ICP (optional)
     */
    bool matchWithModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& detected_cone,
                        Eigen::Matrix4f& transformation);
};

} // namespace cones

#endif // CONES_RECOGNIZER_H
