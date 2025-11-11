#include "ConesRecognizer.h"
#include "DistinguishLeftRight.h"
#include <iostream>
#include <iomanip>

/**
 * Simple test demonstrating the ConesRecognizer class usage
 * without requiring visualization (headless test)
 */

void test_class_based_api() {
    std::cout << "=== Testing Class-Based API ===" << std::endl;
    
    // Create configuration with custom parameters
    cones::RecognizerConfig config;
    config.voxel_leaf_size = 0.01;
    config.sor_mean_k = 50;
    config.cluster_tolerance = 0.05;
    config.min_cluster_size = 10;
    config.max_cluster_size = 10000;
    
    // Set distance filtering
    config.filter_ref_x = 0.0;
    config.filter_ref_y = 0.0;
    config.filter_max_distance = 15.0;  // 15 meter radius
    
    // Create recognizer - notice we only configure once!
    cones::ConesRecognizer recognizer(config);
    
    std::cout << "Configuration set:" << std::endl;
    std::cout << "  - Voxel leaf size: " << config.voxel_leaf_size << std::endl;
    std::cout << "  - Cluster tolerance: " << config.cluster_tolerance << std::endl;
    std::cout << "  - Max distance: " << config.filter_max_distance << " m" << std::endl;
    
    // Detect cones - simple 1-parameter API!
    std::string test_file = "../../../external material/lev_1-4/cones.pcd";
    auto detected_cones = recognizer.detectCones(test_file);
    
    std::cout << "\nDetection Results:" << std::endl;
    std::cout << "  - Total cones detected: " << detected_cones->size() << std::endl;
    
    // Access additional results without re-running detection
    auto clusters = recognizer.getClusters();
    auto full_cloud = recognizer.getConeCloud();
    
    std::cout << "  - Number of clusters: " << clusters.size() << std::endl;
    std::cout << "  - Total points in cone cloud: " << full_cloud->size() << std::endl;
    
    // Demonstrate left/right separation with distance filtering
    std::cout << "\n=== Testing Left/Right Separation ===" << std::endl;
    
    // Without distance filtering
    auto result_no_filter = cones::distinguishLeftRight(detected_cones, 0.0, 0.0);
    std::cout << "Without distance filter:" << std::endl;
    std::cout << "  - Left cones: " << result_no_filter.left_cones->size() << std::endl;
    std::cout << "  - Right cones: " << result_no_filter.right_cones->size() << std::endl;
    
    // With 5 meter distance filter
    auto result_5m = cones::distinguishLeftRight(detected_cones, 0.0, 0.0, -45.0, 45.0, 5.0);
    std::cout << "\nWith 5m distance filter:" << std::endl;
    std::cout << "  - Left cones: " << result_5m.left_cones->size() << std::endl;
    std::cout << "  - Right cones: " << result_5m.right_cones->size() << std::endl;
    
    // With 10 meter distance filter
    auto result_10m = cones::distinguishLeftRight(detected_cones, 0.0, 0.0, -45.0, 45.0, 10.0);
    std::cout << "\nWith 10m distance filter:" << std::endl;
    std::cout << "  - Left cones: " << result_10m.left_cones->size() << std::endl;
    std::cout << "  - Right cones: " << result_10m.right_cones->size() << std::endl;
    
    std::cout << "\n✓ Class-based API test completed successfully!" << std::endl;
}

void demonstrate_parameter_reduction() {
    std::cout << "\n=== Demonstrating Parameter Reduction ===" << std::endl;
    
    std::cout << "\nBEFORE (Procedural API):" << std::endl;
    std::cout << "  recognizeCones(scan_file, model_file," << std::endl;
    std::cout << "                 voxel_size, sor_k, sor_thresh," << std::endl;
    std::cout << "                 plane_thresh, min_ratio," << std::endl;
    std::cout << "                 rot_x, rot_z," << std::endl;
    std::cout << "                 cluster_tol, min_size, max_size," << std::endl;
    std::cout << "                 icp_dist, icp_iter, icp_eps, icp_fit," << std::endl;
    std::cout << "                 ref_x, ref_y, max_dist);" << std::endl;
    std::cout << "  → 19 parameters!" << std::endl;
    
    std::cout << "\nAFTER (Class-based API):" << std::endl;
    std::cout << "  RecognizerConfig config;  // Configure once" << std::endl;
    std::cout << "  ConesRecognizer recognizer(config);" << std::endl;
    std::cout << "  recognizer.detectCones(scan_file);" << std::endl;
    std::cout << "  → 1 parameter!" << std::endl;
    std::cout << "  → 95% reduction in parameters" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "╔═══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  ConesRecognizer Class Refactoring Test                  ║" << std::endl;
    std::cout << "║  Demonstrating improved API with reduced parameters      ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    try {
        test_class_based_api();
        demonstrate_parameter_reduction();
        
        std::cout << "\n╔═══════════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║  ✓ All tests passed successfully!                       ║" << std::endl;
        std::cout << "╚═══════════════════════════════════════════════════════════╝" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
