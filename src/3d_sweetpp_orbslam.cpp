
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
#include <geometry_msgs/TransformStamped.h>
//
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "project.h"
 

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
    // using PointT = pcl::PointXYZRGB;
    // using NormalT = pcl::PointXYZRGBNormal;
    // using PointCloud::Ptr = pcl::PointCloud<PointT>::Ptr;
    //  using PointCloudT1Ptr = pcl::PointCloud<PointT>::Ptr;
  
      ~ImageGrabber () {}
    double depthScale;
    double cx,cy,fx,fy;
    double icp_max_corres_dist ;
    double icp_transformation_epsilon, icp_max_iterations;
    PointCloud::Ptr cloud_rgb, cloud_smt,cloud_stem, cloud_petiole, cloud_leaf, cloud_fruit, cloud_flower,cloud_branch;
    std::vector<PointCloud::Ptr> vt_cloudnormal;
    std::vector<PointCloud::Ptr>  vt_cloudrgb;
    std::vector<PointCloud::Ptr>list_peotiole;
    std::vector<PointT>list_peotiole_pose;
    Eigen::Vector3d camerapose ;
    Eigen::Vector3d point1 , point2 ;
    Eigen::Quaterniond q;
    int fl_creat_3d;
    cv::Mat final_smt;
    PointCloud::Ptr pointcloud_stem;
    float normal_resolution_  ;
    float rgb_resolution_  ;
    int is_view;
    public:
        int n ;
        PointCloud::Ptr pc;
        int change;
        ros::ServiceClient client , client_cut;
        ros::Publisher pub, pub2, pubgeo,pubgeo1;
        cv::Mat color1, color2, depth1,depth2;
        Eigen::Isometry3d T0, T1;

        pcl::visualization::CloudViewer *viewer;
    private:
    // pcl::search::KdTree<PointT, pcl::KdTreeFLANN<PointT, flann::L2_Simple<float>>>::Ptr kdtree_;
        pcl::PointCloud<PointT>::Ptr source_cloud_;
        pcl::PointCloud<PointT>::Ptr target_cloud_;
        Eigen::Matrix4d transformation_;
    public:
        // void estimateNormals (const PointCloud::Ptr& cloud);
      

    public:
    std::vector<PointCloud::Ptr>list_pc;
    std::vector<Eigen::Vector3d >list_c;
    Shape_color ls[6];
     
   
    ImageGrabber( )
    {
        cx  =  	        328.879272460938;
        cy  =         	242.60676574707;
        fx  =          	601.466491699219;
        fy  =          	601.211242675781;
        // depthScale = 0.00025;
            depthScale = 0.001;
        ls[0].id = 0; ls[0].r = 0; ls[0].g = 0; ls[0].b = 0; 
        ls[1].id = 1; ls[1].r = 128; ls[1].g = 64; ls[1].b = 128;   //stem
        ls[2].id = 2; ls[2].r = 0; ls[2].g = 255; ls[2].b = 0;      //leaf
        ls[3].id = 3; ls[3].b = 160; ls[3].g = 170; ls[3].r = 250;  //petiole
        ls[4].id = 4; ls[4].r = 232; ls[4].g = 30; ls[4].b = 244;   //branch
        ls[5].id = 5; ls[5].r = 60; ls[5].g = 20; ls[5].b = 220;    //fruit
        ls[6].id = 6; ls[6].r = 0; ls[6].g = 16; ls[6].b = 172;     //orange rect
        viewer = new pcl::visualization::CloudViewer ("viewer");
        normal_resolution_ = 0.003;
        rgb_resolution_ = 0.005;
        camerapose = Eigen::Vector3d(-0.11, -0.14,0.28);
        fl_creat_3d = 1;
        
        change = 0;
        n = 0;
         pointcloud_stem =  PointCloud::Ptr(new PointCloud);
        pc = PointCloud::Ptr(new PointCloud);
        T0 = Eigen::Isometry3d::Identity();
        T1 = Eigen::Isometry3d::Identity();
        //////////////////////////////////////////////////////////
        cloud_smt = PointCloud::Ptr(new PointCloud);
        cloud_rgb = PointCloud::Ptr(new PointCloud);
        cloud_stem =  PointCloud::Ptr(new PointCloud);
        cloud_petiole =  PointCloud::Ptr(new PointCloud);
        cloud_leaf =  PointCloud::Ptr(new PointCloud);
        cloud_fruit =  PointCloud::Ptr(new PointCloud);
        cloud_branch =  PointCloud::Ptr(new PointCloud);
        cloud_flower = PointCloud::Ptr(new PointCloud);
        source_cloud_ = PointCloud::Ptr(new PointCloud);
        target_cloud_ = PointCloud::Ptr(new PointCloud);
        ///
        icp_max_corres_dist = 0.03;
        icp_transformation_epsilon =  1e-8 ;
        //  # 0.001 meter and 0.256 degree
        icp_max_iterations = 30;
    }
    Eigen::Matrix4d runpcl()
    {
         transformation_ = PCLICP (target_cloud_,source_cloud_ );;
        pcl::PointCloud<PointT>::Ptr source_cloud_transformed (new pcl::PointCloud<PointT>);
        pcl::transformPointCloud (*target_cloud_, *source_cloud_transformed, transformation_);
        // source_cloud_transformed->swap(*source_cloud_);
        return transformation_;
    }
    Eigen::Matrix4d PCLICP (const PointCloud::Ptr& source, const PointCloud::Ptr& target)
    {
        pcl::PointCloud<PointT> registration_output;
        pcl::IterativeClosestPoint<PointT, PointT, double> icp;
        icp.setMaxCorrespondenceDistance (icp_max_corres_dist);
        icp.setTransformationEpsilon (icp_transformation_epsilon);
        icp.setMaximumIterations (icp_max_iterations);
        icp.setInputSource (source_cloud_);
        icp.setInputTarget (target_cloud_);
        icp.align (registration_output);
        return icp.getFinalTransformation ();
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


    Eigen::Isometry3d convertmatrix (Eigen::Matrix4d _transformation_)
    {
            Eigen::Matrix<double,3,3> R1_0;
            
                R1_0 << _transformation_(0,0) , _transformation_(0,1) ,  _transformation_(0,2) ,
                    _transformation_(1,0) , _transformation_(1,1) ,  _transformation_(1,2) ,
                    _transformation_(2,0), _transformation_(2,1) ,  _transformation_(2,2)  ;
            Eigen::Matrix<double,3,1> t1(_transformation_(3,0) , _transformation_(3,1) ,  _transformation_(3,2));
            Eigen::Isometry3d T1(R1_0);
            T1.pretranslate(t1);
            return T1;
            
    }
        /// Convert a ROS Point to an Eigen vector.
    inline Eigen::Vector3d toEigen(geometry_msgs::Point const & vector) {
        return Eigen::Vector3d(vector.x, vector.y, vector.z);
    }

    /// Convert a ROS Point32 to an Eigen vector.
    inline Eigen::Vector3f toEigen(geometry_msgs::Point32 const & vector) {
        return Eigen::Vector3f(vector.x, vector.y, vector.z);
    }

    /// Convert a ROS Vector3 to an Eigen vector.
    inline Eigen::Vector3d toEigen(geometry_msgs::Vector3 const & vector) {
        return Eigen::Vector3d(vector.x, vector.y, vector.z);
    }

    /// Convert a ROS Quaternion to an Eigen quaternion.
    inline Eigen::Quaterniond toEigen(geometry_msgs::Quaternion const & quaternion) {
        return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    }

    Eigen::Isometry3d toEigen(geometry_msgs::Pose const & pose) {
        return translate(toEigen(pose.position)) * toEigen(pose.orientation);
    }
    /// Create a translation from a vector.
    inline Eigen::Translation3d translate(Eigen::Vector3d translation) {
    return Eigen::Translation3d{translation};
    }

    /// Create a translation from X, Y and Z components.
    inline Eigen::Translation3d translate(double x, double y, double z) {
    return translate(Eigen::Vector3d{x, y, z});
    }

    void voxelize2(PointCloud::Ptr pc_src, pcl::PointCloud<PointT>pc_dst, double var_voxel_size){
        if(pc_src->points.size() == 0)
        return;
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (pc_src);
        sor.setMeanK (20);
        sor.setStddevMulThresh (1.0);
        sor.filter (pc_dst);
        pc_src->points  = pc_dst.points;

        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pc_src);
        voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
        voxel_filter.filter(pc_dst);
        pc_src->points  = pc_dst.points;
        
    }
    void voxelize3(PointCloud1::Ptr pc_src, PointCloud1 pc_dst, double var_voxel_size){
        if(pc_src->points.size() == 0)
        return;
        pcl::StatisticalOutlierRemoval<PointT> sor;
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pc_src);
        voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
        voxel_filter.filter(pc_dst);
        pc_src->points  = pc_dst.points;

        sor.setInputCloud (pc_src);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (pc_dst);
        pc_src->points  = pc_dst.points;
    }
    void voxelize4(PointCloud1::Ptr pc_src, PointCloud1 pc_dst, double var_voxel_size){
        if(pc_src->points.size() == 0)
        return;
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pc_src);
        voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
        voxel_filter.filter(pc_dst);
        pc_src->points  = pc_dst.points;
        
    }
    void voxelize5(PointCloud1::Ptr pc_src, PointCloud1 pc_dst, double var_voxel_size){
        if(pc_src->points.size() == 0)
        return;

        pcl::StatisticalOutlierRemoval<PointT> sor;
        pcl::VoxelGrid<PointT> voxel_filter;

        sor.setInputCloud (pc_src);
        sor.setMeanK (30);
        sor.setStddevMulThresh (1.0);
        sor.filter (pc_dst);
        pc_src->points  = pc_dst.points;

        voxel_filter.setInputCloud(pc_src);
        voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
        voxel_filter.filter(pc_dst);
        pc_src->points  = pc_dst.points;

    
        
        
    }
    void voxelize1(PointCloud1::Ptr pc_src, PointCloud1 pc_dst, double var_voxel_size){
        if(pc_src->points.size() == 0)
        return;
        
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pc_src);
        voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
        voxel_filter.filter(pc_dst);
        pc_src->points  = pc_dst.points;
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (pc_src);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (pc_dst);
        pc_src->points  = pc_dst.points;

    }
    PointCloud::Ptr create_colorpointcloud(cv::Mat &color, cv::Mat &depth)
    {
        PointCloud::Ptr current(new PointCloud);
        current->points.clear();
        for (int v = 0; v < color.rows; v+=3)
        {    for (int u = 0; u < color.cols; u+=3) 
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                Eigen::Vector3d point;
                point[2] = double(d) * depthScale;
                if (point[2]== 0 || point[2] >2) continue;
                point[0] = (u - cx ) * point[2] / fx ;
                point[1] = (v - cy ) * point[2] / fy ;
                Eigen::Vector3d pointWorld ;
                pointWorld = T1.inverse() * point;
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
    PointCloud::Ptr create_smtpointcloud(cv::Mat &color, cv::Mat &depth   )
    {
        PointCloud::Ptr current(new PointCloud);
        current->points.clear();
        for (int v = 0; v < color.rows; v+=2)
        {    for (int u = 0; u < color.cols; u+=2) 
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                Eigen::Vector3d point;
                point[2] = double(d) * depthScale;
                if (point[2]== 0 || point[2] >2) continue;
                point[0] = (u - cx ) * point[2] / fx ;
                point[1] = (v - cy ) * point[2] / fy ;
                Eigen::Vector3d pointWorld ;
                pointWorld = T1.inverse() * point;
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
        for (int v = 0; v < color.rows; v+=2)
        {    for (int u = 0; u < color.cols; u+=2) 
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                Eigen::Vector3d point;
                point[2] = double(d) * depthScale;
                if (point[2]== 0 || point[2] >2) continue;
                point[0] = (u - cx ) * point[2] / fx ;
                point[1] = (v - cy ) * point[2] / fy ;
                Eigen::Vector3d pointWorld ;
                pointWorld = T1.inverse() * point;
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
    void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgSmt,
                const sensor_msgs::ImageConstPtr& msgD,const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr,  cv_depth ;
        cv::Mat rgb, smt , depth_img;
        cout <<"\n in callback function";
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::RGB8);
            rgb = cv_ptr->image;
            cv_ptr = cv_bridge::toCvCopy(msgSmt, sensor_msgs::image_encodings::TYPE_8UC3);
            smt = cv_ptr->image;
            // cv::cvtColor(smt, smt, CV_BGR2RGB);
            cv_depth = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_16UC1); //sensor_msgs::image_encodings::MONO16
            depth_img = cv_depth->image;

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if(fl_creat_3d == 0)
            return;
        create_3d_pointcloud(rgb,smt, depth_img, toEigen(msg->pose) );
    }
    void create_3d_pointcloud(cv::Mat rgb,cv::Mat smt,cv::Mat depth, Eigen::Isometry3d pose)
    {
 
         
        int j = 0;
        PointCloud::Ptr currentrgb(new PointCloud);
        PointCloud::Ptr currentsmt(new PointCloud);
        PointCloud::Ptr currentstem(new PointCloud);
        PointCloud::Ptr currentpetiole(new PointCloud);
        PointCloud::Ptr currentfruit(new PointCloud);
        PointCloud::Ptr currentleaf(new PointCloud);
        PointCloud::Ptr pc_smt(new PointCloud);
        PointCloud::Ptr pc_rgb(new PointCloud);
        float _resolution_ = 0.003;
        int re = 10e4;
        int key_size = 10; // number of images in a unit lof list
        int in = 0;
        // T1 = toEigen(msg->pose);
        T1 =  pose ;
        Eigen::Vector3d d1 (1,1,1),d4 (0,0,0),d2 (0,0,0),d3 (1,1,1);
        d2 = T1 * d1;
        d4 = T0 * d3;
        T0 =  T1;
        source_cloud_->clear();
        pc->clear();
      
        // *currentstem = *create_type_pointcloud(smt,depth, 1);  voxelize3(currentstem,*currentstem,normal_resolution_);
        // *currentleaf = *create_type_pointcloud(smt,depth, 2);  voxelize3(currentleaf,*currentleaf,normal_resolution_);
        // *currentpetiole = *create_type_pointcloud(smt,depth, 3);  voxelize3(currentpetiole,*currentpetiole,normal_resolution_);
        // *currentfruit = *create_type_pointcloud(smt,depth, 5);  voxelize3(currentfruit,*currentfruit,normal_resolution_);
        for (int v = 0; v < smt.rows; v+=2)
        {    
            for (int u = 0; u < smt.cols; u+=2) 
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; 
                Eigen::Vector3d point;
                point[2] = double(d) * depthScale;
                if (point[2]<0.5 || point[2] >2.5) continue;
                point[0] = (u - cx ) * point[2] / fx ;
                point[1] = (v - cy ) * point[2] / fy ;
                Eigen::Vector3d pointWorld ;
                pointWorld = T1.inverse() * point;
            
                PointT p2;
                p2.x = pointWorld[0];
                p2.y = pointWorld[1];
                p2.z = pointWorld[2];
                p2.b = rgb.data[v * rgb.step + u * rgb.channels()];
                p2.g = rgb.data[v * rgb.step + u * rgb.channels() + 1];
                p2.r = rgb.data[v * rgb.step + u * rgb.channels() + 2];
                currentrgb->points.push_back(p2);
                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = smt.data[v * smt.step + u * smt.channels()];
                p.g = smt.data[v * smt.step + u * smt.channels() + 1];
                p.r = smt.data[v * smt.step + u * smt.channels() + 2];
                currentsmt->points.push_back(p);
                if( p.r == ls[1].r && p.g == ls[1].g && p.b == ls[1].b) //if stem
                {
                    currentstem->points.push_back(p);
                    
                }
                else
                if( p.r == ls[2].r  && p.g == ls[2].g && p.b == ls[2].b ) //if leaf
                {
                    currentleaf->points.push_back(p);
                     
                }
                else
                    if( p.r == ls[3].r  && p.g == ls[3].g && p.b ==ls[3].b ) //if petiole
                    {
                        currentpetiole->points.push_back(p);
                        
                    }
                    else
                        if( p.r == ls[5].r  && p.g == ls[5].g  && p.b == ls[5].b ) //if fruit
                        {
                            currentfruit->points.push_back(p);
                        }

            }
        }
        
        currentstem->is_dense = false;
        currentleaf->is_dense = false;
        currentpetiole->is_dense = false;
        currentfruit->is_dense = false;
        
        voxelize2(currentstem,*currentstem,normal_resolution_);
        voxelize2(currentleaf,*currentleaf,normal_resolution_);
        voxelize2(currentpetiole,*currentpetiole, _resolution_);
        voxelize2(currentfruit,*currentfruit,normal_resolution_);
        
        *pc_smt = *currentsmt;
        voxelize3(pc_smt,*pc_smt,normal_resolution_);
        cout <<"\n orginal currentsmt size()" <<pc_smt->points.size();
        if (pc_smt->points.size() <50)
        {
            // n--;
            return;
        }
        if(pc_smt->points.size() > 2000)
        {
            normal_resolution_ = 2 *normal_resolution_;
            voxelize2(currentsmt,*currentsmt,normal_resolution_);
        }  
        else
        {
            if(pc_smt->points.size() <=400)
            {
                normal_resolution_ = normal_resolution_ /2;
            }
            voxelize2(currentsmt,*currentsmt,normal_resolution_);
        }
        voxelize3(currentrgb,*currentrgb,rgb_resolution_);

        n++;

        if(n > 3) //bat dau so sanh va tim transform
        {
          
            int change = 1;
            if(currentsmt->points.size() > 400)
              change = 0;
            else 
              cout << "\n Dung rgb de so sanh";
         
            if(change)
            {
                cout <<"\n smt cloud size: "<< currentsmt->points.size() ;
                cout <<"\n current normal_resolution_: " << normal_resolution_;
                cout <<"\n **** Transform by RGB cloud ";
                *source_cloud_+= *vt_cloudrgb[0]  +*cloud_rgb;
                voxelize2(source_cloud_, *source_cloud_, rgb_resolution_);
                *target_cloud_ = *currentrgb;
            }
            else
            {
                pc_smt->clear();
               *source_cloud_+= *vt_cloudnormal[0]   + *cloud_stem  ;//+ *cloud_smt;
                voxelize2(source_cloud_, *source_cloud_, normal_resolution_);
                if(source_cloud_->points.size() > 5000)
                {
                    voxelize2(source_cloud_,*source_cloud_, 2.5 *normal_resolution_);
                }  
                else
                {
                    if(source_cloud_->points.size() > 2000)
                    {
                        voxelize2(source_cloud_,*source_cloud_, 2 *normal_resolution_);
                    }  
                }
                *target_cloud_ = *currentsmt;
            }
            std::cout << "\nsource_cloud_ size:" << source_cloud_->size();
            std::cout << "\ntarget_cloud_ size:" << target_cloud_->size();
            transformation_ = runpcl();
            Eigen::Isometry3d Ta = convertmatrix(transformation_);
            pcl::transformPointCloud (*currentstem, *currentstem,transformation_ );
            pcl::transformPointCloud (*currentsmt, *currentsmt,transformation_ );
            pcl::transformPointCloud (*currentrgb, *currentrgb,transformation_ );
            pcl::transformPointCloud (*currentleaf, *currentleaf,transformation_ );
            pcl::transformPointCloud (*currentpetiole, *currentpetiole,transformation_ );
            pcl::transformPointCloud (*currentfruit, *currentfruit,transformation_ );
         
            cv::imshow("rgb", rgb);
            cv::imshow(OPENCV_WINDOW, smt);
            cv::waitKey(1);
        }
        if(n  == 1)
        {
            vt_cloudnormal.push_back(currentsmt);
            vt_cloudrgb.push_back(currentrgb);

        }
        if (n == 2)
        {
            vt_cloudnormal.push_back(currentsmt);
            vt_cloudrgb.push_back(currentrgb);
            *vt_cloudnormal[0] += *vt_cloudnormal[1]; 
            *vt_cloudrgb[0] += *vt_cloudrgb[1];
        }
        if (n == 3)
        {
            vt_cloudnormal.push_back(currentsmt);
            vt_cloudrgb.push_back(currentrgb);
            *vt_cloudnormal[0] += *vt_cloudnormal[1]+*currentsmt ; 
            *vt_cloudrgb[0] += *vt_cloudrgb[1] +*currentsmt;
            *vt_cloudnormal[1] += *vt_cloudnormal[2];
            *vt_cloudrgb[1] += *vt_cloudrgb[2];
        }
        if(n > 3)
        {
            vt_cloudnormal.erase(vt_cloudnormal.begin());
            vt_cloudrgb.erase(vt_cloudrgb.begin());
            vt_cloudnormal.push_back(currentsmt);
            vt_cloudrgb.push_back(currentrgb);
             *vt_cloudnormal[0] += *vt_cloudnormal[1]+*currentsmt ; 
            *vt_cloudrgb[0] += *vt_cloudrgb[1] +*currentsmt;
            *vt_cloudnormal[1] += *vt_cloudnormal[2];
            *vt_cloudrgb[1] += *vt_cloudrgb[2];
        }
        *cloud_smt += *currentsmt;
        *cloud_rgb += *currentrgb;
        *cloud_stem += *currentstem;
        *cloud_leaf += *currentleaf;
        *cloud_petiole += *currentpetiole;
        *cloud_fruit += *currentfruit;
        voxelize3(cloud_rgb, *cloud_rgb, rgb_resolution_);
        if(n % 3 == 0)
        {
            voxelize3(cloud_smt, *cloud_smt,normal_resolution_);  
        }
        
        if(n % 10 == 0)
        {
             voxelize3(cloud_leaf, *cloud_leaf, normal_resolution_);
            // voxelize3(cloud_stem, *cloud_stem, normal_resolution_);
            // voxelize5(cloud_petiole, *cloud_petiole, normal_resolution_);
            // voxelize3(cloud_fruit, *cloud_fruit, normal_resolution_);
            
        }
        
        cout <<"\n n: " <<n;
        PointCloud1::Ptr pc_show(new PointCloud1);
        *pc_show = *cloud_stem + *cloud_leaf + *cloud_petiole;
        viewer->showCloud (pc_show, "body" );
    }
};
  
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "read_image_bag");
    ros::NodeHandle node_handler;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/semantic/rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/semantic/depth", 1);
    message_filters::Subscriber<sensor_msgs::Image> smt_sub(node_handler, "/semantic/smt", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub (node_handler, "/semantic/camera", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,geometry_msgs::PoseStamped> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1), rgb_sub,smt_sub, depth_sub,pose_sub);
    ImageGrabber igb;
    sync.registerCallback(boost::bind(&ImageGrabber::callback,&igb, _1, _2,_3,_4));
    ros::spin();
    return 0;
}