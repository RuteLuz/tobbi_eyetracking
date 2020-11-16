#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

#include <sstream>
#include <tobii/tobii.h>
#include <tobii/tobii_streams.h>
#include <cstdio>
#include <cassert>
#include <cstring>

class Gazing(int argc, char **argv){
    public:
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher eyetracking_pub_new = n.advertise<geometry_msgs::Point>("eye_tracking_position", 1000);
    ros::Subscriber sub_new = n.subscribe("input", 1000, &Gazing::chatterCallback);

    void chatterCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("I heard: %s", msg->data.c_str());
    }
};



static void url_receiver(char const *url, void *user_data) {
    char *buffer = (char *) user_data;
    if (*buffer != '\0') return; // only keep first value

    if (strlen(url) < 256)
        strcpy(buffer, url);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
//    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("eye_tracking_position", 1000);
    ros::Publisher eyetracking_pub = n.advertise<geometry_msgs::Point>("eye_tracking_position", 1000);
    ros::Subscriber sub = n.subscribe("input", 1000, chatterCallback);

    ros::Rate loop_rate(10);
    int count = 0;

    ROS_INFO("Up and running...");

    tobii_api_t *api;
    tobii_error_t error = tobii_api_create(&api, NULL, NULL);
    assert(error == TOBII_ERROR_NO_ERROR);

    char url[256] = {0};
    error = tobii_enumerate_local_device_urls(api, url_receiver, url);
    assert(error == TOBII_ERROR_NO_ERROR && *url != '\0');

    tobii_device_t *device;
    error = tobii_device_create(api, url, &device);
    assert(error == TOBII_ERROR_NO_ERROR);

    error = tobii_gaze_point_subscribe(device, gaze_point_callback, 0);
    assert(error == TOBII_ERROR_NO_ERROR);

    while (ros::ok())
    {
        error = tobii_wait_for_callbacks(1, &device);
        assert(error == TOBII_ERROR_NO_ERROR || error == TOBII_ERROR_TIMED_OUT);

        error = tobii_device_process_callbacks(device);
        assert(error == TOBII_ERROR_NO_ERROR);
/*
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        chatter_pub.publish(msg);
        ++count;
*/
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}