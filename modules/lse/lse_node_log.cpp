#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/system_identification.h"

using namespace std;

void subscriberCallback(FILE*, const surgical_robot::system_identificationConstPtr &);
typedef const boost::function<void(const surgical_robot::system_identificationConstPtr &)> sub_callback;

int main (int argc, char **argv){
    ros::init(argc,argv,"lse_log");
    ros::NodeHandle n;
    
    //file, audomatically increase
    system("mkdir -p ${HOME}/catkin_ws/src/surgical_robot/modules/lse/data");
    int num = 0;
    string prefix = "data_";
    string suffix = ".csv";
    string name;
    string dir = "/home/earsuit/catkin_ws/src/surgical_robot/modules/lse/data/";
    string path;
    FILE* fp = NULL;
    do{
        if(fp !=NULL)
            fclose(fp);     
        num++;
        name = prefix + to_string(num)+suffix;
        path = dir+name;
        fp = fopen(path.c_str(),"r");
    }while(fp != NULL);
    fp = fopen(path.c_str(),"w");
    fprintf(fp,"Angle,current,Voltage\n");

    //subscriber 
    sub_callback subscriber_callback = boost::bind(subscriberCallback,fp,_1);
    ros::Subscriber lse_log_sub = n.subscribe("system_identification",1000,subscriber_callback);

    ros::spin();

    fclose(fp);

    return 0;
}

void subscriberCallback(FILE* fp, const surgical_robot::system_identificationConstPtr & msg){
    ROS_DEBUG("%f,%f,%f\n",msg->motor_angle,msg->motor_v,msg->current);
    fprintf(fp,"%f,%f,%f\n",msg->motor_angle,msg->motor_v,msg->current);
}
