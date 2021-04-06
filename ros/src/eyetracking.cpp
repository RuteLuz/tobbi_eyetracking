#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

#include <sstream>
#include <tobii/tobii.h>
#include <tobii/tobii_streams.h>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <fstream>

using namespace std;
using std::cout;
using std::cin;
using std::endl;
using std::string;

ros::Publisher eyetracking_pub;
const char *file_name;

void gaze_point_callback(tobii_gaze_point_t const *gaze_point, void *user_data) {
    if (gaze_point->validity == TOBII_VALIDITY_VALID){
        std::ofstream fout(file_name, std::ios::out|std::ios::app);
        fout<<gaze_point->position_xy[0]<<","<<gaze_point->position_xy[1]<<"\n";

        geometry_msgs::Point eye_position;
        eye_position.x = gaze_point->position_xy[0];
        eye_position.y = gaze_point->position_xy[1];
        eyetracking_pub.publish(eye_position);
    }
}

static void url_receiver(char const *url, void *user_data) {
    char *buffer = (char *) user_data;
    if (*buffer != '\0') return; // only keep first value

    if (strlen(url) < 256)
        strcpy(buffer, url);
}

int main(int argc, char **argv)
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"/home/rute/ros_ws/src/tobii_eyetracking/ros/recorded_data/%Y_%m_%d_%H:%M.txt",timeinfo);
    file_name =  buffer;

    ros::init(argc, argv, "eye_tracker_node");
    ros::NodeHandle n;
    eyetracking_pub = n.advertise<geometry_msgs::Point>("eye_tracking_position", 1000);

    ros::Rate loop_rate(10);

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

    error = tobii_gaze_point_subscribe(device, gaze_point_callback, NULL);
    assert(error == TOBII_ERROR_NO_ERROR);

    while (ros::ok())
    {
        error = tobii_wait_for_callbacks(1, &device);
        assert(error == TOBII_ERROR_NO_ERROR || error == TOBII_ERROR_TIMED_OUT);

        error = tobii_device_process_callbacks(device);
        assert(error == TOBII_ERROR_NO_ERROR);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
