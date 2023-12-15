
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
struct Shape_color{
    int id;
    int r,g,b;
};

class ImageGrabber
{
    public:
    std::vector<PointCloud::Ptr>list_pc;
    std::vector<Eigen::Vector3d >list_c;
    Shape_color ls[6];
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
        ls[0].id = 0; ls[0].r = 0; ls[0].g = 0; ls[0].b = 0; 
        ls[1].id = 1; ls[1].r = 128; ls[1].g = 64; ls[1].b = 128;   //blue circle
        ls[2].id = 2; ls[2].r = 0; ls[2].g = 255; ls[2].b = 0;      //blue rect
        ls[3].id = 3; ls[3].r = 160; ls[3].g = 170; ls[3].b = 250;  //green circle
        ls[4].id = 4; ls[4].r = 232; ls[4].g = 30; ls[4].b = 244;   //green rect
        ls[5].id = 5; ls[5].r = 60; ls[5].g = 20; ls[5].b = 220;    //orange circle
        ls[6].id = 6; ls[6].r = 0; ls[6].g = 16; ls[6].b = 172;     //orange rect
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

    PointCloud::Ptr createtarget_colorpointcloud(cv::Mat &color, cv::Mat &depth)
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
    PointCloud::Ptr createtarget_smtpointcloud(cv::Mat &color, cv::Mat &depth   )
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
    PointCloud::Ptr create_type_pointcloud(cv::Mat &color, cv::Mat &depth,  int index  )
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
                // if( p.r != 0 && p.g != 0 && p.b != 0)
                //     cout <<"\n"<< (int)p.r <<" - " <<(int)p.g <<" - "<<(int)p.b;
                if( (int)p.r  == ls[index].r && (int)p.g == ls[index].g && (int)p.b == ls[index].b)
                    current->points.push_back(p);
                
            }
        }

        return current;
    }

    void show_3d(cv::Mat rgb, cv::Mat depth)
    {
        PointCloud::Ptr pt = createtarget_colorpointcloud(rgb,depth );
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
        PointCloud::Ptr pt = createtarget_smtpointcloud(smt,depth );
        voxelize(pt,*pt,0.001);
        // cout << "number of points: " << pt->points.size();
        viewer->showCloud (pt, "body" );
        
    }
    double distance(PointT a, PointT b)
    {
        float kq = sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) + pow(a.z - b.z,2));
        return kq;
    }

    PointT getequal(PointT a)
    {
        PointT b; 
        b.x = a.x;
        b.y = a.y;
        b.z = a.z;
        b.g = a.g;
        b.b = a.b;
        b.r = a.r;
        return b;
    }
   
    PointCloud::Ptr count_group( PointCloud::Ptr tmp, float radius, float threshold)
    {
        // list_peotiole_pose
        // list_peotiole
        int K = 2;
        int g = 0;
        PointCloud::Ptr tmp_f(new PointCloud);
        PointCloud::Ptr tmp_r1(new PointCloud);
        while ( tmp->size() > 0)
        {
            pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
            pcl::PointXYZRGB searchPoint;
            kdtree.setInputCloud (tmp);
            searchPoint.x = 0;
            searchPoint.y = 0;
            searchPoint.z = 0;
            tmp_r1->points.clear();
            std::vector<int> pointIdxKNNSearch(K);
            std::vector<float> pointKNNSquaredDistance(K);
        
            if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) <=1 )
                break;
            searchPoint.x = (*tmp)[ pointIdxKNNSearch[1] ].x;
            searchPoint.y = (*tmp)[ pointIdxKNNSearch[1] ].y;
            searchPoint.z = (*tmp)[ pointIdxKNNSearch[1] ].z;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            std::vector<pcl::PointXYZRGB> vfind;
            int n = 0;
            double max_x = -100, max_y = -100, max_z = -100;
            double min_x = 100, min_y = 100, min_z = 100;
            do 
            {
                kdtree.setInputCloud (tmp);
                if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
                {
                    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                    { 
                    
                        pcl::PointXYZRGB rt = getequal((*tmp)[ pointIdxRadiusSearch[i] ]);
                        if(distance(searchPoint, rt) > radius) //kiem tra lai radius vi ham tren tinh radius sai
                            continue;
                        pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
                        for (it = tmp->begin(); it != tmp->end();)
                        {
                            bool remove_point = 0;
                            if ( it->z == rt.z && it->y == rt.y && it->x == rt.x     )
                            {
                                n++;
                                it = tmp->erase(it);
                                tmp_r1->push_back(rt );
                                vfind.push_back(rt);
                                break;
                                
                            }
                            else
                            {
                                it++;
                            }
                        }
                    }
                }
                if (vfind.size() > 0)
                {
                    searchPoint.x = vfind[0].x;
                    searchPoint.y = vfind[0].y;
                    searchPoint.z = vfind[0].z;
                    vfind.erase(vfind.begin());
                }
                
            }while(vfind.size() > 0 && tmp->size()>0);
            if(n > threshold && tmp_r1->size() > 0)
            {
                PointCloud::Ptr tmp_r2(new PointCloud);
                *tmp_r2 = *tmp_r1;
                list_pc.push_back(tmp_r2);
                *tmp_f += *tmp_r1;
            } 
        }
        return tmp_f;
    }
    
     Eigen::Vector3d find_the_center_point(PointCloud::Ptr tmp)
    {
        Eigen::Vector3d c(0,0,0);
        double x = 0, y = 0, z = 0;
        int count = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
        for (it = tmp->begin(); it != tmp->end();)
        {
            x += it->x;
            y += it->y;
            z += it->z;
            count ++;
            it++;
        }
        c[0] = x/count;
        c[1] = y/count;
        c[2] = z/count;
        return c;
    }

    void show_the_center_point(PointCloud::Ptr pc_color)
    {
        list_pc.clear();
        list_c.clear();
        count_group( pc_color, 0.004, 50);
        PointCloud::Ptr pc_color_center(new PointCloud);
        for(int i = 0; i< list_pc.size(); i++)
        {
            *pc_color_center += *list_pc[i];
            Eigen::Vector3d c = find_the_center_point(list_pc[i]);
            list_c.push_back(c);
            cout <<"\n center point "<<i<<": "<< c[0] <<" " <<c[1]<<" " <<c[2];
            for(int k = -1; k < 2; k++)
            {
                for(int t = -1; t <2 ; t++)
                {
                    for(int f = -1; f<2 ; f++)
                    {
                        PointT p  ;
                        p.x = c[0] + k* 0.001;
                        p.y = c[1] + t* 0.001;
                        p.z = c[2] + f* 0.001;;
                        p.b = 255;
                        p.g = 255;
                        p.r = 255;
                        pc_color_center->points.push_back(p);
                    }
                }
            }
        }
        viewer->showCloud (pc_color_center, "body" );
    }
    public:
    void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
    {
        cv_bridge::CvImagePtr cv_ptr,  cv_depth ;
        cv::Mat smt , depth_img;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::TYPE_8UC3);
            smt = cv_ptr->image;
            // cv::cvtColor(smt, smt, CV_BGR2RGB);
            cv_depth = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_img = cv_depth->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        PointCloud::Ptr pt = createtarget_smtpointcloud(smt,depth_img );
        voxelize(pt,*pt,0.001);
        viewer->showCloud (pt, "body" );
        cv::imshow("color", smt);
        cv::waitKey(0);
        PointCloud::Ptr pc_blue_cir(new PointCloud);
        pc_blue_cir = create_type_pointcloud(smt,depth_img ,1);
        viewer->showCloud (pc_blue_cir, "body" );
        cv::waitKey(0);
        show_the_center_point(pt);
        cv::waitKey(0);
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