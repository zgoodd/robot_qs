/*
 * 该程序用来订阅下位机发布的图像数据  可以用 rostopic list 查看
 *
 */


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;










void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try   // 如果转换失败，则提跳转到catch语句
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);   // 将图像转换openCV的格式，并输出到窗口
        cv::waitKey(1); // 一定要有wiatKey(),要不然是黑框或者无窗口     
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert for '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    ROS_INFO("222");































    
}

int main(int argc, char** argv)
{
    ROS_INFO("111");
    ros::init(argc, argv, "imageSub_node");   // 注册节点名 
    ros::NodeHandle nh; // 注册句柄
    image_transport::ImageTransport it(nh); // 注册句柄

    cv::namedWindow("view");
    cv::startWindowThread();

    image_transport::Subscriber imageSub = it.subscribe("/cameraImage", 1, imageCallback);  // 订阅/cameraImage话题，并添加回调函数
    ros::spin();  // 循环等待回调函数触发
    cv::destroyWindow("view");  //窗口
}
