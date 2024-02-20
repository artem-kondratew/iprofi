#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


class Roi {
private:
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber image_sub;

    cv_bridge::CvImagePtr cv_ptr;

public:
    Roi() {
        image_pub = nh.advertise<sensor_msgs::Image>("/head/camera1/roi", 1);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        image_sub = nh.subscribe("/head/camera1/image_raw", 1, &Roi::camera_cb, this);
    
        // ros::Duration(1).sleep();
    }

    void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
        size_t w = msg->width;
        size_t h = msg->height;
        size_t size = w * h;
        size_t channels = 3;

        cv::Mat rgb(h, w, CV_8UC3);
        std::memcpy(rgb.data, msg->data.data(), sizeof(uint8_t) * size * channels);

        int x = 100;
        int y = 0;
        int roi_w = w - 2 * x;
        int roi_h = 340;
        int roi_size = roi_w * roi_h;

        cv::Rect roi_rect = cv::Rect(x, y, roi_w, roi_h);
        cv::Mat roi = rgb(roi_rect).clone();
        // cv::rectangle(rgb, roi_rect, {255, 0, 255}, 2);

        //cv::imshow("rgb", rgb);
        //cv::imshow("roi", roi);
        //cv::waitKey(20);

        auto pub_msg = sensor_msgs::Image();
        pub_msg.header.frame_id = msg->header.frame_id;
        pub_msg.header.stamp = msg->header.stamp;

        pub_msg.height = roi_h;
        pub_msg.width = roi_w;

        pub_msg.encoding = msg->encoding;
        pub_msg.is_bigendian = msg->is_bigendian;
        pub_msg.step = roi_w * channels;

        pub_msg.data.resize(roi_size * channels);
        std::memcpy(pub_msg.data.data(), roi.data, sizeof(uint8_t) * roi_size * channels);

        image_pub.publish(pub_msg);
    }

    void spin() {
        double t0 = ros::Time::now().toSec();
        while (nh.ok()) {
            double t = ros::Time::now().toSec() - t0;

            geometry_msgs::Twist command;

            // command.linear.x = 0.0;
            // command.linear.y = 0.0;
            // command.angular.z = 0.5;

            // cmd_vel_pub.publish(command);

            ros::spinOnce();
        }
    }

};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "example_node");

    Roi ex;
    ex.spin();

    return 0;
}
