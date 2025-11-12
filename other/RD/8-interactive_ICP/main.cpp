#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f |\n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f |\n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f |\n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}





int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input_scan.pcd> <model_cone.ply>" << std::endl;
        return -1;
    }

    std::string pcd_file = argv[1];
    std::string ply_file = argv[2];

    // Load LiDAR scan (source)
    PointCloudT::Ptr cloud_in(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud_in) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file.\n");
        return -1;
    }

    // Load model cone scene (target)
    PointCloudT::Ptr cloud_model(new PointCloudT);
    if (pcl::io::loadPLYFile<PointT>(ply_file, *cloud_model) == -1)
    {
        PCL_ERROR("Couldn't read the PLY file.\n");
        return -1;
    }

    std::cout << "Loaded " << cloud_in->size() << " points from " << pcd_file << std::endl;
    std::cout << "Loaded " << cloud_model->size() << " points from " << ply_file << std::endl;

    // --- Center both clouds around their centroid ---
    Eigen::Vector4f centroid_in, centroid_model;
    pcl::compute3DCentroid(*cloud_in, centroid_in);
    pcl::compute3DCentroid(*cloud_model, centroid_model);

    Eigen::Matrix4f T_center_in = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_center_model = Eigen::Matrix4f::Identity();

    T_center_in(0, 3) = -centroid_in[0];
    T_center_in(1, 3) = -centroid_in[1];
    T_center_in(2, 3) = -centroid_in[2];

    T_center_model(0, 3) = -centroid_model[0];
    T_center_model(1, 3) = -centroid_model[1];
    T_center_model(2, 3) = -centroid_model[2];

    pcl::transformPointCloud(*cloud_in, *cloud_in, T_center_in);
    pcl::transformPointCloud(*cloud_model, *cloud_model, T_center_model);

    std::cout << "Both point clouds centered around their barycenter.\n";

    // Prepare for ICP
    PointCloudT::Ptr cloud_aligned(new PointCloudT);
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_model);

    pcl::console::TicToc time;
    time.tic();
    icp.align(*cloud_aligned);
    std::cout << "ICP alignment completed in " << time.toc() << " ms\n";

    if (!icp.hasConverged())
    {
        PCL_ERROR("ICP did not converge.\n");
        return -1;
    }

    std::cout << "\nICP converged with score = " << icp.getFitnessScore() << std::endl;

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation);

    // --- Compute mean Euclidean distance between aligned scan and model ---
    double total_distance = 0.0;
    int count = 0;

    for (const auto &p : cloud_aligned->points)
    {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto &q : cloud_model->points)
        {
            double dx = p.x - q.x;
            double dy = p.y - q.y;
            double dz = p.z - q.z;
            double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < min_dist)
                min_dist = d2;
        }
        total_distance += std::sqrt(min_dist);
        ++count;
    }

    double mean_distance = (count > 0) ? total_distance / count : 0.0;
    std::cout << "\nAverage Euclidean distance between aligned scan and model: "
              << mean_distance << " meters\n";

    // --- Visualization ---
    pcl::visualization::PCLVisualizer viewer("ICP Alignment (Centered)");
    int v1(0), v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> model_color(cloud_model, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> scan_color(cloud_in, 20, 180, 20);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> aligned_color(cloud_aligned, 180, 20, 20);

    viewer.addPointCloud(cloud_model, model_color, "model_v1", v1);
    viewer.addPointCloud(cloud_in, scan_color, "scan_v1", v1);

    viewer.addPointCloud(cloud_model, model_color, "model_v2", v2);
    viewer.addPointCloud(cloud_aligned, aligned_color, "aligned_v2", v2);

    viewer.addText("Left: Initial alignment (centered)\nRight: After ICP", 10, 15, "text", v1);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.setBackgroundColor(0, 0, 0, v2);
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        if (next_iteration)
        {
            time.tic();
            icp.align(*cloud_aligned);
            std::cout << "1 more ICP iteration: score = " << icp.getFitnessScore()
                      << " in " << time.toc() << " ms\n";
            viewer.updatePointCloud(cloud_aligned, aligned_color, "aligned_v2");
            next_iteration = false;
        }
    }

    return 0;
}
