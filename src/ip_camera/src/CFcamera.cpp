#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char** argv)
{

    cv::VideoCapture cap;
    cap.open("rtsp://192.168.50.50:554/Streaming/Channels/101?transportmode=unicast");
    if(!cap.isOpened())
    {
        //如果视频不能正常打开则返回
         return -1;
    }
    else
    {
        ROS_INFO("Camera open success");
    }
    
    cv::Mat frame;

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    
    ros::Rate loop_rate(5);
    while (nh.ok()) {
        cap>>frame;
        // cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}