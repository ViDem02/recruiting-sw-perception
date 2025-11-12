#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

int main(int argc, char *argv[])
{
    PointCloudT::Ptr cloud_in(new PointCloudT);   // first.pcd (target)
    PointCloudT::Ptr cloud_src(new PointCloudT);  // second.pcd (source)
    PointCloudT::Ptr cloud_aligned(new PointCloudT); // final aligned source

    int iterations = 200; // sensible default
    if (argc > 1)
    {
        try { iterations = std::stoi(argv[1]); } catch (...) {
            PCL_ERROR("Failed to parse iterations.\n");
            return -1;
        }
        if (iterations < 1)
        {
            PCL_ERROR("Iterations must be >= 1\n");
            return -1;
        }
    }

    pcl::console::TicToc time; time.tic();
    if (pcl::io::loadPCDFile("first.pcd", *cloud_in) < 0)
    { PCL_ERROR("Error loading first.pcd\n"); return -1; }
    if (pcl::io::loadPCDFile("second.pcd", *cloud_src) < 0)
    { PCL_ERROR("Error loading second.pcd\n"); return -1; }
    std::cout << "Loaded first.pcd (" << cloud_in->size() << ") and second.pcd (" << cloud_src->size() << ") in " << time.toc() << " ms\n\n";

    // Run ICP once for the requested iterations.
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_in);
    icp.setMaximumIterations(iterations);
    time.tic();
    icp.align(*cloud_aligned);
    double ms = time.toc();

    if (!icp.hasConverged())
    {
        PCL_ERROR("ICP did not converge.\n");
        return -1;
    }

    Eigen::Matrix4d final_transform = icp.getFinalTransformation().cast<double>();
    std::cout << "ICP converged in " << ms << " ms with score " << icp.getFitnessScore() << "\n";
    std::cout << "Final transformation (second -> first):\n";
    print4x4Matrix(final_transform);

    // Visualization (static)
    pcl::visualization::PCLVisualizer viewer("ICP Result: second.pcd aligned to first.pcd");
    int vp_left(0), vp_right(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, vp_left);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, vp_right);

    float bg = 0.0f; float txt = 1.0f;
    viewer.setBackgroundColor(bg, bg, bg, vp_left);
    viewer.setBackgroundColor(bg, bg, bg, vp_right);

    // Color handlers
    pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_color(cloud_in, 255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_color(cloud_src, 20,180,20);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> aligned_color(cloud_aligned, 200,30,30);

    viewer.addPointCloud(cloud_in, tgt_color, "cloud_in_left", vp_left);
    viewer.addPointCloud(cloud_src, src_color, "cloud_src_left", vp_left);
    viewer.addPointCloud(cloud_in, tgt_color, "cloud_in_right", vp_right);
    viewer.addPointCloud(cloud_aligned, aligned_color, "cloud_aligned_right", vp_right);

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_left");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_src_left");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_right");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_aligned_right");

    // Overlay text
    std::ostringstream info_left;
    info_left << "Target: first.pcd (white)\nSource: second.pcd (green)\nIterations: " << iterations;
    viewer.addText(info_left.str(), 10, 15, 14, txt, txt, txt, "info_left", vp_left);

    std::ostringstream info_right;
    info_right << "Result (red) vs target (white)\nScore: " << icp.getFitnessScore();
    viewer.addText(info_right.str(), 10, 15, 14, txt, txt, txt, "info_right", vp_right);

    // Transformation text (single line summary)
    std::ostringstream tf_line;
    tf_line << "Translation: (" << final_transform(0,3) << ", " << final_transform(1,3) << ", " << final_transform(2,3)
            << ")";
    viewer.addText(tf_line.str(), 10, 60, 14, txt, txt, txt, "tf_line", vp_right);

    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);

    // Static visualization loop (no animation)
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(30);
    }
    return 0;
}
