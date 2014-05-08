#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>


#include <vtkRenderWindow.h>



ros::Publisher pub;
ros::Publisher pub2;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

// PCL Viewer
bool refreshUI = true;
int l_count = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

void printToPCLViewer(){
    pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"source cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(cloud_p, 255, 0, 0);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(cloud_p,red_color,"segmented cloud");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");

}


// Callback Function for the subscribed ROS topic
void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input){
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud

    pcl::fromPCLPointCloud2(*input,*cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (input);
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    pcl::PCLPointCloud2 cloud_filtered;
    sor.filter (cloud_filtered);

    // Create the segmentation object
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);


    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_filtered,*cloud_filtered2);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);


    seg.setInputCloud (cloud_filtered2);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    // Extract the inliers
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Publish the model coefficients
    pcl::PCLPointCloud2 segmented_pcl;
    pcl::toPCLPointCloud2(*cloud_p,segmented_pcl);
    sensor_msgs::PointCloud2 segmented, not_segmented;
    pcl_conversions::fromPCL(cloud_filtered,segmented);
    pcl_conversions::fromPCL(*input,not_segmented);
    pub.publish (segmented);
    pub2.publish(not_segmented);

    //Update PCL Viewer
    printToPCLViewer();

}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_planes", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/extracted_planes_not", 1);


    //PCL Viewer
     pclViewer->setBackgroundColor (0, 0, 0);
     pclViewer->initCameraParameters ();
     pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
     vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
     renderWindow->SetSize(800,450);
     renderWindow->Render();

     ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        ros::spinOnce();
       // boost::this_thread::sleep (boost::posix_time::microseconds (10000));
        r.sleep();
    }
    return 0;
}
