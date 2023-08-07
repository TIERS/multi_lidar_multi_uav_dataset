 #include <time.h>

 // Include ROS library
#include <ros/ros.h>

// Include TF for transforms
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// Include point clouds types
#include <sensor_msgs/PointCloud2.h> 
 
// Include PCL library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h> 

#include <fstream>
#include <chrono>
#include <string>
#include <Eigen/Dense>
  
typedef  pcl::PointXYZ   PointType;
 
class ExtrinsicCalibrator{
    private: 
        ros::NodeHandle nh;

        ros::Subscriber sub_avia;
        ros::Subscriber sub_mid360;
        ros::Subscriber sub_os;
        ros::Subscriber sub_camera;

        ros::Publisher pub_avia;
        ros::Publisher pub_mid360;
        ros::Publisher pub_os;
        ros::Publisher pub_camera;

        tf::TransformBroadcaster tf_br;

        // Avia tf
        int avia_integrate_frames = 5;
        pcl::PointCloud<PointType> avia_igcloud;
        Eigen::Matrix4f avia_tf_matrix; 
        bool avia_tf_initd = false;

        // Mid-360 TF
        int mid360_integrate_frames = 5;
        pcl::PointCloud<PointType> mid360_igcloud;
        Eigen::Matrix4f mid360_tf_matrix; 
        bool mid360_tf_initd = false;

        // Camera TF
        int camera_integrate_frames = 5;
        Eigen::Matrix4f camera_tf_matrix; 
        bool camera_tf_initd = false;
        pcl::PointCloud<PointType> camera_igcloud;

        // Ouster TF
        Eigen::Matrix4f os_tf_matrix; 
        bool os_tf_initd = false;
        pcl::PointCloud<PointType> os_cloud;
        bool os_received = false;

        // Cropbox filter to limit points distance
        pcl::CropBox<PointType> cropBoxFilter;
        Eigen::Vector4f min_box_filter;
        Eigen::Vector4f max_box_filter;


    public:
        ExtrinsicCalibrator(){ 
            sub_avia = nh.subscribe<sensor_msgs::PointCloud2>("/avia_points", 1000, &ExtrinsicCalibrator::aviaCallback, this);
            sub_mid360 = nh.subscribe<sensor_msgs::PointCloud2>("/mid360_points", 1000, &ExtrinsicCalibrator::mid360Callback, this);
            sub_os = nh.subscribe<sensor_msgs::PointCloud2>("/ouster/points", 1000, &ExtrinsicCalibrator::ousterCallback, this);
            sub_camera = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1000, &ExtrinsicCalibrator::cameraCallback, this);
 
            pub_avia    = nh.advertise<sensor_msgs::PointCloud2>("/a_avia", 1);
            pub_mid360    = nh.advertise<sensor_msgs::PointCloud2>("/a_mid360", 1);
            pub_os     = nh.advertise<sensor_msgs::PointCloud2>("/a_os", 1);
            pub_camera     = nh.advertise<sensor_msgs::PointCloud2>("/a_camera", 1);

            // Set min and max box filter
            min_box_filter = Eigen::Vector4f(-50.0, -50.0, -50.0, 1.0);
            max_box_filter = Eigen::Vector4f(50.0, 50.0, 50.0, 1.0);
            
            cropBoxFilter.setMin(min_box_filter);
            cropBoxFilter.setMax(max_box_filter);
        };
         

        ~ExtrinsicCalibrator(){};

        void aviaCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            if(!os_received) return;
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header); 
             
            if(avia_integrate_frames > 0)
            {
                avia_igcloud += full_cloud_in; 
                avia_integrate_frames--; 
                return;
            }else
            {
                if(!avia_tf_initd){

                    ROS_INFO("\n\n\n  Calibrate Avia ...");
                    calibratePointCloud(avia_igcloud.makeShared(), os_cloud.makeShared(), avia_tf_matrix); 
                    Eigen::Matrix3f rot_matrix = avia_tf_matrix.block(0,0,3,3);
                    Eigen::Vector3f trans_vector = avia_tf_matrix.block(0,3,3,1);

                    std::cout << "Avia -> base_link " << trans_vector.transpose()
                        << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "avia_frame"
                        << " /" << "base_link" << " 10" << std::endl;

                    // publish result
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (avia_igcloud , out_cloud, avia_tf_matrix);

                    sensor_msgs::PointCloud2 avia_msg;
                    pcl::toROSMsg(out_cloud, avia_msg);
                    avia_msg.header.stamp = ros::Time::now();
                    avia_msg.header.frame_id = "base_link"; 
                    pub_avia.publish(avia_msg); 

                    avia_tf_initd = true;
                }else
                {        
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (full_cloud_in , out_cloud, avia_tf_matrix);

                    sensor_msgs::PointCloud2 avia_msg;
                    pcl::toROSMsg(out_cloud, avia_msg);
                    avia_msg.header.stamp = ros::Time::now();
                    avia_msg.header.frame_id = "base_link"; 
                    pub_avia.publish(avia_msg); 
                }
            } 
        }
 
        void mid360Callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            if(!os_received) return;
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header);
             
            if(mid360_integrate_frames > 0)
            {
                mid360_igcloud += full_cloud_in;
                mid360_integrate_frames--;
                return;
            }else
            {
                if(!mid360_tf_initd){
 
                    ROS_INFO("\n\n\n  Calibrate Mid-360 ...");
                    calibratePointCloud(mid360_igcloud.makeShared(), os_cloud.makeShared(), mid360_tf_matrix);
                    Eigen::Matrix3f rot_matrix = mid360_tf_matrix.block(0,0,3,3);
                    Eigen::Vector3f trans_vector = mid360_tf_matrix.block(0,3,3,1);
        
                    std::cout << "Mid360 -> base_link " << trans_vector.transpose()
                        << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "mid360_frame"
                        << " /" << "base_link" << " 10" << std::endl;

                    // publish result
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (mid360_igcloud , out_cloud, mid360_tf_matrix);

                    sensor_msgs::PointCloud2 mid360_msg;
                    pcl::toROSMsg(out_cloud, mid360_msg);
                    mid360_msg.header.stamp = ros::Time::now();
                    mid360_msg.header.frame_id = "base_link";
                    pub_mid360.publish(mid360_msg);

                    mid360_tf_initd = true;
                }else
                {        
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (full_cloud_in , out_cloud, mid360_tf_matrix);

                    sensor_msgs::PointCloud2 mid360_msg;
                    pcl::toROSMsg(out_cloud, mid360_msg);
                    mid360_msg.header.stamp = ros::Time::now();
                    mid360_msg.header.frame_id = "base_link"; 
                    pub_mid360.publish(mid360_msg); 
                }
            }
        }

        void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header);

            if(camera_integrate_frames > 0)
            {
                camera_igcloud += full_cloud_in; 
                camera_integrate_frames--;
                return;
            }else
            {
                if(!camera_tf_initd){

                    Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
                    Eigen::AngleAxisf init_rot_y( 0.0, Eigen::Vector3f::UnitY());
                    Eigen::AngleAxisf init_rot_z( 0.0 , Eigen::Vector3f::UnitZ());
                    Eigen::Translation3f init_trans(0.0,0.0,0.0);
                    Eigen::Matrix4f camera_tf_matrix = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();
 
                    ROS_INFO("\n\n\n  Calibrate Camera ...");
                    // calibratePointCloud(camera_igcloud.makeShared(), os_cloud.makeShared(), camera_tf_matrix);
                    Eigen::Matrix3f rot_matrix = camera_tf_matrix.block(0,0,3,3);
                    Eigen::Vector3f trans_vector = camera_tf_matrix.block(0,3,3,1);
        
                    std::cout << "Camera -> base_link " << trans_vector.transpose()
                        << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "camera_depth_optical_frame"
                        << " /" << "base_link" << " 10" << std::endl;

                    // publish result
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (full_cloud_in , full_cloud_in, camera_tf_matrix);

                    sensor_msgs::PointCloud2 camera_msg;
                    pcl::toROSMsg(camera_igcloud, camera_msg);
                    camera_msg.header.stamp = ros::Time::now();
                    camera_msg.header.frame_id = "base_link"; 
                    pub_camera.publish(camera_msg); 

                    camera_tf_initd = true;
                }else
                {        
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (full_cloud_in , out_cloud, camera_tf_matrix);

                    sensor_msgs::PointCloud2 camera_msg;
                    pcl::toROSMsg(camera_igcloud, camera_msg);
                    camera_msg.header.stamp = ros::Time::now();
                    camera_msg.header.frame_id = "base_link"; 
                    pub_camera.publish(camera_msg); 
                }
            }
  
        }

        void ousterCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header);
             
            Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf init_rot_z( 0.0 , Eigen::Vector3f::UnitZ());
            Eigen::Translation3f init_trans(0.0,0.0,0.0);
            Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();

            Eigen::Matrix3f rot_matrix = init_tf.block(0,0,3,3);
            Eigen::Vector3f trans_vector = init_tf.block(0,3,3,1);

            if(!os_tf_initd){
                 std::cout << "OS0 -> base_link " << trans_vector.transpose()
                    << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "os0_sensor"
                    << " /" << "base_link" << " 10" << std::endl;

                os_tf_initd = true;
            }
            
            pcl::PointCloud<PointType>  out_cloud;
            pcl::transformPointCloud (full_cloud_in , full_cloud_in, init_tf);
            
            os_cloud.clear();
            os_cloud += full_cloud_in;  
            os_received = true;

            sensor_msgs::PointCloud2 os_msg;
            pcl::toROSMsg(os_cloud, os_msg);
            os_msg.header.stamp = ros::Time::now();
            os_msg.header.frame_id = "base_link"; 
            pub_os.publish(os_msg); 
  
        }

        void calibratePointCloud(pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud, Eigen::Matrix4f &tf_matrix)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::ratio<1, 1000>> time_span =
            std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

            std::cout << "------------checking PCL GICP---------------- "<< std::endl;
            int pCount = source_cloud->size();

            pcl::PointCloud<PointType>::Ptr source_cloud_downsampled (new pcl::PointCloud<PointType> );
            pcl::PointCloud<PointType>::Ptr target_cloud_downsampled (new pcl::PointCloud<PointType> );

            // Remove far points
            pcl::PointCloud<PointType>::Ptr source_cloud_filtered (new pcl::PointCloud<PointType>);

            // Crop point cloud to a max distance of 50 meters
            cropBoxFilter.setInputCloud(source_cloud);
            cropBoxFilter.filter(*source_cloud_filtered);

            ROS_INFO_STREAM("Source-> Removing far points "<< source_cloud->size() << " "<<  source_cloud_filtered->size() );
 
            // Remove far points
            pcl::PointCloud<PointType>::Ptr target_cloud_filtered (new pcl::PointCloud<PointType>);  

            cropBoxFilter.setInputCloud(target_cloud);
            cropBoxFilter.filter(*target_cloud_filtered);
            ROS_INFO_STREAM("Target-> Removing far points "<< target_cloud->size() << " "<<  target_cloud_filtered->size() );

            // Apply voxel grid filter to downsample cloud
            pcl::VoxelGrid< PointType> vox;
            vox.setLeafSize (0.05f, 0.05f, 0.05f);

            vox.setInputCloud (source_cloud_filtered);
            vox.filter(*source_cloud_downsampled);

            vox.setInputCloud (target_cloud_filtered);
            vox.filter(*target_cloud_downsampled);

            std::cout << "GICP start  .... " << source_cloud_downsampled->size() << " to "<< target_cloud_downsampled->size()<< std::endl;
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
            // gicp.setTransformationEpsilon(0.001);
            // gicp.setMaxCorrespondenceDistance(2);
            // gicp.setMaximumIterations(500);
            // gicp.setRANSACIterations(12);  
            // gicp.setInputSource(source_cloud_downsampled);
            // gicp.setInputTarget(target_cloud_downsampled);
            gicp.setTransformationEpsilon(0.01);
            gicp.setMaxCorrespondenceDistance(10.0);
            gicp.setMaximumIterations(100);
            gicp.setRANSACIterations(100);  
            gicp.setInputSource(source_cloud_downsampled);
            gicp.setInputTarget(target_cloud_downsampled);

            pcl::PointCloud<PointType>::Ptr aligned_cloud (new pcl::PointCloud<PointType>);

            t1 = std::chrono::steady_clock::now();
            gicp.align(*aligned_cloud);
            t2 = std::chrono::steady_clock::now();
            time_span = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
            std::cout << "has converged: " << gicp.hasConverged() << " score: " <<
                gicp.getFitnessScore() << std::endl; 

            tf_matrix =  gicp.getFinalTransformation ();
        } 
};

int 
main(int argc, char **argv)
{  
    ros::init(argc, argv, "Extrinsic Calibration"); 

    ExtrinsicCalibrator swo; 
    ROS_INFO("\033[1;32m---->\033[0m Extrinsic Calibration Started.");  

    ros::Rate r(10);
    while(ros::ok()){   

        ros::spinOnce(); 
        r.sleep(); 
  
    }


    return 0;
}
