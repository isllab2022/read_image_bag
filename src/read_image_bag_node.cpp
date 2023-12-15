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
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
class ImageGrabber
{
    public:
    void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
    {
        cv_bridge::CvImagePtr cv_ptr,  cv_depth ;
        cv::Mat rgb_img , depth_img;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::RGB8);
            rgb_img = cv_ptr->image;
            cv_depth = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_img = cv_depth->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        string datadir = "/home/isl/catkin_ws/src/read_image_bag/images";
        auto timestamp = timeSinceEpochMillisec();
        boost::format fmt("%s/%s/%d.png");
        cv::cvtColor(rgb_img, rgb_img, cv::COLOR_RGB2BGR);
        cv::imshow("T435_1", rgb_img);  
        cv::waitKey(1);
         
        if(timestamp % 4 == 0)
        {
            cv::imwrite ((fmt % datadir %"rgb" %(timestamp)).str(),rgb_img);
            cv::imwrite((fmt % datadir %"depth"  %(timestamp)).str(),depth_img);
            cout<<"\nwrite image "<<(fmt % datadir %"rgb" %(timestamp)).str();
        }
    }
};
  
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "read_image_bag");
    ros::NodeHandle node_handler;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    ImageGrabber igb;
    sync.registerCallback(boost::bind(&ImageGrabber::callback,&igb, _1, _2));
    ros::spin();
    return 0;
}
