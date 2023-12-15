#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include "read_image_bag/Rgbimage.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
class Viewdepth{
    public:
    
        std::string path_rgb, path_depth;
        cv::Mat rgb, depth, smt;
        double cx,cy,fx,fy,depthScale;
        pcl::visualization::CloudViewer *viewer;
        Viewdepth(std::string m_path_rgb,std::string m_path_depth)
        {
            cx  =  	        328.879272460938;
            cy  =         	242.60676574707;
            fx  =          	601.466491699219;
            fy  =          	601.211242675781;
            depthScale = 0.00025;
            path_rgb = m_path_rgb;
            path_depth = m_path_depth;
            viewer = new pcl::visualization::CloudViewer ("viewer");
        }
        void load_image()
        {
             rgb = cv::imread(path_rgb,cv::IMREAD_UNCHANGED);
             depth = cv::imread(path_depth,cv::IMREAD_UNCHANGED);
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

        void show_3d()
        {
            PointCloud::Ptr pt = createtarget_colorpointcloud(rgb,depth,depthScale,cx,cy,fx,fy);
            // voxelize(pt,*pt,0.001);
            cout << "number of points: " << pt->points.size();
            cv::imshow("color", rgb);
            cv::imshow("depth", depth);
            viewer->showCloud (pt, "body" );
        }
         void show_smt3d()
        {
            cv::imshow("color", smt);
            PointCloud::Ptr pt = createtarget_smtpointcloud(smt,depth,depthScale,cx,cy,fx,fy);
            voxelize(pt,*pt,0.001);
            cout << "number of points: " << pt->points.size();
            viewer->showCloud (pt, "body" );
        }

};

int main(int argc, char* argv[])
{
    std::string path1 = "/home/isl/catkin_ws/src/read_image_bag/images/rgb/1701666479916.png";
    std::string path2 = "/home/isl/catkin_ws/src/read_image_bag/images/depth/1701666479916.png";
    Viewdepth vd =  Viewdepth(path1,path2);
    vd.load_image();
    vd.show_3d();
    cv::waitKey(0);

    ros::init(argc, argv, "read_image_bag_3d_node");
    ros::NodeHandle node_handler;
    ros::ServiceClient client_srv = node_handler.serviceClient<read_image_bag::Rgbimage>("/rgbshape_model");
    read_image_bag::Rgbimage srv;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",vd.rgb).toImageMsg();
    srv.request.s_input = *image_msg;
    if (client_srv.call(srv))
    {
        cv_bridge::CvImagePtr cv_semantic = cv_bridge::toCvCopy(srv.response.s_ouput, sensor_msgs::image_encodings::TYPE_8UC3);
        vd.smt = cv_semantic->image;
        vd.show_smt3d();
    } 
    else
    {
        cout << "\n errror " << client_srv.isValid();
    }
   
    cv::waitKey(0);
   
}

 