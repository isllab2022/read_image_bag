
#include <iostream>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/format.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <cmath>
#include "read_image_bag/Rgbimage.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
 

class ImageGrabber
{
       public:
    
        // std::string path_rgb, path_depth;
        // cv::Mat rgb, depth, smt;
        double cx,cy,fx,fy,depthScale;
        pcl::visualization::CloudViewer *viewer;
        ImageGrabber( )
        {
            cx  =  	        328.879272460938;
            cy  =         	242.60676574707;
            fx  =          	601.466491699219;
            fy  =          	601.211242675781;
            // depthScale = 0.00025;
             depthScale = 0.001;
            viewer = new pcl::visualization::CloudViewer ("viewer");
        }
       
        void voxelize(PointCloud::Ptr pc_src, PointCloud pc_dst, double var_voxel_size){
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud (pc_src);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.filter (pc_dst);
            pc_src->points  = pc_dst.points;
            pcl::VoxelGrid<PointT> voxel_filter;
            voxel_filter.setInputCloud(pc_src);
            voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
            voxel_filter.filter(pc_dst);
            pc_src->points  = pc_dst.points;
        }

        PointCloud::Ptr createtarget_colorpointcloud(cv::Mat &color, cv::Mat &depth,  
                    double depthScale, double cx, double cy, double fx, double fy)
        {
            PointCloud::Ptr current(new PointCloud);
            current->points.clear();
            for (int v = 0; v < color.rows; v++)
            {    for (int u = 0; u < color.cols; u++) 
                {
                    unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                    Eigen::Vector3d point;
                    point[2] = double(d) * depthScale;
                    if (point[2]== 0 || point[2] >2) continue;
                    point[0] = (u - cx ) * point[2] / fx ;
                    point[1] = (v - cy ) * point[2] / fy ;
                    Eigen::Vector3d pointWorld ;
                    pointWorld =  point;
                    PointT p;
                    p.x = pointWorld[0];
                    p.y = pointWorld[1];
                    p.z = pointWorld[2];
                    p.b = color.data[v * color.step + u * color.channels()];
                    p.g = color.data[v * color.step + u * color.channels() + 1];
                    p.r = color.data[v * color.step + u * color.channels() + 2];
                    current->points.push_back(p);
                }
            }
            return current;
        }
        PointCloud::Ptr createtarget_smtpointcloud(cv::Mat &color, cv::Mat &depth,  
                    double depthScale, double cx, double cy, double fx, double fy)
        {
            PointCloud::Ptr current(new PointCloud);
            current->points.clear();
            for (int v = 0; v < color.rows; v++)
            {    for (int u = 0; u < color.cols; u++) 
                {
                    unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                    Eigen::Vector3d point;
                    point[2] = double(d) * depthScale;
                    if (point[2]== 0 || point[2] >2) continue;
                    point[0] = (u - cx ) * point[2] / fx ;
                    point[1] = (v - cy ) * point[2] / fy ;
                    Eigen::Vector3d pointWorld ;
                    pointWorld =  point;
                    PointT p;
                    p.x = pointWorld[0];
                    p.y = pointWorld[1];
                    p.z = pointWorld[2];
                    p.b = color.data[v * color.step + u * color.channels()];
                    p.g = color.data[v * color.step + u * color.channels() + 1];
                    p.r = color.data[v * color.step + u * color.channels() + 2];
                    if( p.r == 0 && p.g == 0 && p.b == 0)
                      continue;
                    current->points.push_back(p);
                }
            }
            return current;
        }

        void show_3d(cv::Mat rgb, cv::Mat depth)
        {
            PointCloud::Ptr pt = createtarget_colorpointcloud(rgb,depth,depthScale,cx,cy,fx,fy);
            // voxelize(pt,*pt,0.001);
            // cout << "number of points: " << pt->points.size();
            cv::imshow("color", rgb);
            cv::imshow("depth", depth);
            viewer->showCloud (pt, "body" );
             cv::waitKey(0);
        }
        void show_smt3d(cv::Mat smt, cv::Mat depth)
        {
            cv::imshow("color", smt);
            PointCloud::Ptr pt = createtarget_smtpointcloud(smt,depth,depthScale,cx,cy,fx,fy);
            voxelize(pt,*pt,0.001);
            // cout << "number of points: " << pt->points.size();
            viewer->showCloud (pt, "body" );
           
        }
    public:
    void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
    {
        cv_bridge::CvImagePtr cv_ptr,  cv_depth ;
        cv::Mat rgb_img , depth_img;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::TYPE_8UC3);
            rgb_img = cv_ptr->image;
            // cv::cvtColor(rgb_img, rgb_img, CV_BGR2RGB);
            cv_depth = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_img = cv_depth->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        show_smt3d(rgb_img,depth_img);
    }
};
  
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "read_image_bag");
    ros::NodeHandle node_handler;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/semantic/smt", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/semantic/depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    ImageGrabber igb;
    sync.registerCallback(boost::bind(&ImageGrabber::callback,&igb, _1, _2));
    ros::spin();
    return 0;
}