#include <iostream>
#include <string>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>


using namespace std;

typedef pcl::PointXYZRGB RGB_Point;
typedef pcl::PointCloud<RGB_Point> point_cloud;
typedef point_cloud::Ptr cloud_pointer;


void Load_PCDFile(int i, cloud_pointer cloud)
{
    string openFileName;

    // Generate object to store cloud in .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    openFileName = "/home/yxw/Project/test/icp_pointcloud/pcd/Captured_Frame" + to_string(i) + ".pcd";
    pcl::io::loadPCDFile (openFileName, *cloud); // Load .pcd File
    
    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Captured Frame"
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Captured Frame"));

    // // Set background of viewer to black
    // viewer->setBackgroundColor (0, 0, 0); 
    // // Add generated point cloud and identify with string "Cloud"
    // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "Cloud");
    // // Default size for rendered points
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // // Viewer Properties
    // viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    // cout << endl;
    // cout << "Press [Q] in viewer to continue. " << endl;
    
    // viewer->spin(); // Allow user to rotate point cloud and view it

    // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.
   
}

void pc_viewer(cloud_pointer cloud)
{
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Captured Frame"));

    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0); 
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // Viewer Properties
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    cout << endl;
    viewer->spin();
}

void euler2tran(Eigen::Vector3d& euler_angel, Eigen::Vector3d& position, Eigen::Matrix4f& tran)
{
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(euler_angel[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(euler_angel[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(euler_angel[2], Eigen::Vector3d::UnitX());
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            tran(i, j) = rotation_matrix(i, j);
        }
    }

    tran(3, 0) = tran(3, 1) = tran(3, 2) = tran(0,3) = tran(1, 3) = tran(2, 3) = 0;
    tran(3,3) = 1;

    
}



int main()
{
    cloud_pointer cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_pointer cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_pointer cloud_icp (new pcl::PointCloud<pcl::PointXYZRGB>);

    Load_PCDFile(1, cloud1);
    Load_PCDFile(2, cloud2);


    Eigen::Matrix4f T;
            
    euler2tran();
    euler2tran();



    pcl::transformPointCloud(*cloud1, *cloud2, T);

    pcl::IterativeClosestPoint<RGB_Point, RGB_Point> icp;
    icp.setMaximumIterations(1);
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);  

    if (icp.hasConverged()) {
        cout <<  "ICP has converged, score is " << icp.getFitnessScore() << endl;
        // const auto matrix = icp.getFinalTransformation().cast<double>();
        
    } else {
        cout <<  "ICP did not converge" << endl;;
    }

    pc_viewer(cloud_icp);
    return 0;
}