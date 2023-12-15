#ifndef  MY_COLORICP_H
#define  MY_COLORICP_H
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp> // needed for L2 distance; skip PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "icp_color/helper.h"
#include "icp_color/remove_nan.h"
typedef pcl::PointXYZRGB PointT;
 //typedef Pointvalue PointT;
typedef pcl::PointCloud<PointT> PointCloud1;
static const std::string OPENCV_WINDOW = "Image window";


namespace Eigen {
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, -1> Matrix6Xd;
} // namespace Eigen
//dinh nghia lop tinh thoi gian
#endif 
