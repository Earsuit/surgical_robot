#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/system_identification.h"

using namespace std;

void subscriberCallback(FILE*, const surgical_robot::system_identificationConstPtr &);
typedef const boost::function<void(const surgical_robot::system_identificationConstPtr &)> sub_callback;

int main (int argc, char **argv){
    ros::init(argc,argv,"lse_log");
    ros::NodeHandle n;
    
    //file
    system("mkdir -p ${HOME}/catkin_ws/src/surgical_robot/modules/lse/data");
    FILE* fp;
    fp = fopen("/data/data.csv","w");

    //subscriber 
    sub_callback subscriber_callback = boost::bind(subscriberCallback,fp,_1);
    ros::Subscriber lse_log_sub = n.subscribe("system_identification",1000,subscriber_callback);

    ros::spin();

    return 0;
}

void subscriberCallback(FILE* fp, const surgical_robot::system_identificationConstPtr & msg){
    ROS_INFO("%f,%f,%f\n",msg->motor_angle,msg->current,msg->voltage);
    fprintf(fp,"%f,%f,%f\n",msg->motor_angle,msg->current,msg->voltage);
}
