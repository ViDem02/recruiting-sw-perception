#include "ConesRecognizer.h"
#include "DistinguishLeftRight.h"
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

/**
 * Example usage of the refactored ConesRecognizer class
 * and DistinguishLeftRight function
 */

void get_racing_line_point_cloud(
    const std::string& scan_file,
    double robot_x,
    double robot_y,
    double max_distance = std::numeric_limits<double>::infinity())
{
    // Create recognizer with custom configuration
    cones::RecognizerConfig config;
    config.voxel_leaf_size = 0.01;
    config.sor_mean_k = 50;
    config.cluster_tolerance = 0.05;
    config.min_cluster_size = 10;
    config.max_cluster_size = 10000;
    
    // Set distance filtering in recognizer
    config.filter_ref_x = robot_x;
    config.filter_ref_y = robot_y;
    config.filter_max_distance = max_distance;
    
    cones::ConesRecognizer recognizer(config);
    
    // Detect cones - notice how we don't need to pass all parameters!
    auto source_cones = recognizer.detectCones(scan_file);
    
    std::cout << "Detected " << source_cones->size() << " cones" << std::endl;
    
    // Distinguish left and right cones
    // Using default max_distance (infinite) - could also pass max_distance parameter
    auto left_right_res = cones::distinguishLeftRight(
        source_cones,
        robot_x, robot_y);
    
    std::cout << "Left cones: " << left_right_res.left_cones->size() << std::endl;
    std::cout << "Right cones: " << left_right_res.right_cones->size() << std::endl;
    
    // Alternatively, could use max_distance here as well:
    // auto left_right_res = cones::distinguishLeftRight(
    //     source_cones,
    //     robot_x, robot_y,
    //     -45.0, 45.0,  // angle thresholds
    //     max_distance);  // distance threshold
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <scan.pcd> [robot_x] [robot_y] [max_distance]" << std::endl;
        std::cout << "Example: " << argv[0] << " cones.pcd 0.0 0.0 10.0" << std::endl;
        return -1;
    }
    
    std::string scan_file = argv[1];
    double robot_x = (argc > 2) ? std::atof(argv[2]) : 0.0;
    double robot_y = (argc > 3) ? std::atof(argv[3]) : 0.0;
    double max_distance = (argc > 4) ? std::atof(argv[4]) : std::numeric_limits<double>::infinity();
    
    std::cout << "Processing scan: " << scan_file << std::endl;
    std::cout << "Robot position: (" << robot_x << ", " << robot_y << ")" << std::endl;
    
    if (std::isinf(max_distance)) {
        std::cout << "Max distance: infinity (no filtering)" << std::endl;
    } else {
        std::cout << "Max distance: " << max_distance << std::endl;
    }
    
    // Example 1: Using the class-based approach
    std::cout << "\n=== Using ConesRecognizer class ===" << std::endl;
    
    cones::RecognizerConfig config;
    config.filter_ref_x = robot_x;
    config.filter_ref_y = robot_y;
    config.filter_max_distance = max_distance;
    
    cones::ConesRecognizer recognizer(config);
    auto cones = recognizer.detectCones(scan_file);
    
    std::cout << "Detected " << cones->size() << " cone centers" << std::endl;
    
    // Get left/right classification
    auto result = cones::distinguishLeftRight(cones, robot_x, robot_y);
    
    std::cout << "Left cones: " << result.left_cones->size() << std::endl;
    std::cout << "Right cones: " << result.right_cones->size() << std::endl;
    
    // Visualize results
    pcl::visualization::PCLVisualizer viewer("Cone Detection Results");
    
    // Add detected cones
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> all_cones_color(
        cones, 255, 255, 255);
    viewer.addPointCloud(cones, all_cones_color, "all_cones");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "all_cones");
    
    // Add left cones (blue)
    if (result.left_cones->size() > 0) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> left_color(
            result.left_cones, 0, 0, 255);
        viewer.addPointCloud(result.left_cones, left_color, "left_cones");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "left_cones");
    }
    
    // Add right cones (red)
    if (result.right_cones->size() > 0) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> right_color(
            result.right_cones, 255, 0, 0);
        viewer.addPointCloud(result.right_cones, right_color, "right_cones");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "right_cones");
    }
    
    // Add robot position
    pcl::PointXYZ robot_pos;
    robot_pos.x = robot_x;
    robot_pos.y = robot_y;
    robot_pos.z = 0.0;
    viewer.addSphere(robot_pos, 0.5, 0, 255, 0, "robot");
    
    viewer.addCoordinateSystem(1.0);
    viewer.setBackgroundColor(0, 0, 0);
    
    std::cout << "\nVisualization window opened. Close it to exit." << std::endl;
    
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    
    return 0;
}
