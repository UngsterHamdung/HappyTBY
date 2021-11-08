#define DEBUG 0
#define DEBUG_ROS_INFO 1

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>

using namespace std;

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 96

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)
// unit : m
#define Sonar_Detect_Range 0.3
#define LIDAR_Obastacle 0.6
#define LIDAR_Wall_Detection 0.3

int Base_Speed = 130;

//////////////////////////////// Odometry ////////////////////////////
struct BaseSensorData {
    int encoder = 0;
    int encoder_old = 0;
} myBaseSensorData;

struct OdomCaculateData {
    //motor params
    float distance_ratio = (67.6 * 3.14159 / 1000) / 2200; //0.000176; unit: m/encode  67.5, param
    float wheel_distance = 0.226; //0.1495; unit: m
    float encode_sampling_time = 0.05; //unit: s-> 20hz
    float cmd_vel_linear_max = 0.8; //unit: m/s
    float cmd_vel_angular_max = 0.0; //unit: rad/s

    //odom result
    float position_x = 0.0; //unit: m
    float position_y = 0.0; //unit: m
    float oriention = 0.0; //unit: rad
    float velocity_linear = 0.0; //unit: m/s
    float velocity_angular = 0.0; //unit: rad/s
} myOdomCaculateData;

int steering_angle = STEER_NEUTRAL_ANGLE;
int motor_speed = 0;

long encoder = 0;

unsigned long timestamp_now_nsec_old = 0;

void EncoderCallback(const std_msgs::Int16& msg) {
    myBaseSensorData.encoder = msg->data;
}

void CmdVelCallback(const geometry_msgs::Twist & msg) { //radian/s + degree??
    steering_angle = (int)(-msg.angular.z + STEER_NEUTRAL_ANGLE);

    if (steering_angle >= 40 + STEER_NEUTRAL_ANGLE) 
        steering_angle = STEER_NEUTRAL_ANGLE + 40;
    if (steering_angle <= -40 + STEER_NEUTRAL_ANGLE) 
        steering_angle = STEER_NEUTRAL_ANGLE - 40;
    
    Base_Speed = (int)msg.linear.x;
    motor_speed = Base_Speed;
    if (motor_speed >= 250) 
        motor_speed = 250;
    if (motor_speed <= -250) 
        motor_speed = -250;
}

void odometry_cal() {
    int d_encoder = myBaseSensorData.encoder - myBaseSensorData.encoder_old;
    float d_x, d_y, d_r, d_th;

    d_r = d_encoder * myOdomCaculateData.distance_ratio;

    if (motor_speed == 0) {
        d_th = 0;
    }
    else {
        // d_th = DEG2RAD((steering_angle - STEER_NEUTRAL_ANGLE) * 0.008);
        d_th = 
    }
    d_x = d_r * cos(myOdomCaculateData.oriention - d_th);
    d_y = d_r * sin(myOdomCaculateData.oriention - d_th);
    myOdomCaculateData.position_x += d_x; //unit: m
    myOdomCaculateData.position_y += d_y; //unit: m
    myOdomCaculateData.oriention -= d_th; //unit: rad

    ros::Time timestamp_now = ros::Time::now();
    unsigned long timestamp_now_nsec = timestamp_now.toNsec();
    unsigned long d_time = (timestamp_now_nsec - timestamp_now_nsec_old) / 1000000000.0;
    timestamp_now_nsec_old = timestamp_now_nsec;
    
    float d_s = d_r * self.config.encoder_step; // what is encoder step??

    myOdomCaculateData.velocity_linear = d_s/float(d_time);
    myOdomCaculateData.velocity_angular = d_th/float(d_time);

    cout << myOdomCaculateData.position_x << ' ' << myOdomCaculateData.position_y << ' ' << RAD2DEG(myOdomCaculateData.oriention) << '\n';
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "HappyTBY_core");

    ros::NodeHandle n;
    std::string cmd_vel_topic = "cmd_vel";
    std::string odom_pub_topic = "odom";

    ros::Subscriber encoder_sub = n.subscribe("/HappyHBY/HappyTBY_driver/encoder", 10, &EncoderCallback);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, &CmdVelCallback);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
    ros::Publisher motor_pub = n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20); // float steering_angle, velocity, param

    timestamp_now_nsec_old = ros::Time::now().toNsec();

    //////////////////  odometry  ////////////////////
    std::string odom_frame_id = "odom";
    std::string odom_child_frame_id = "base_footprint";

    ros::Rate loop_rate(20); // 10
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    //covariance matrix
    float covariance[36] = {
        0.01,0,0,0,0,0, // covariance on gps_x
        0,0.01,0,0,0,0, // covariance on gps_y
        0,0,99999,0,0,0, // covariance on gps_z
        0,0,0,99999,0,0, // large covariance on rot x
        0,0,0,0,99999,0, // large covariance on rot y
        0,0,0,0,0,0.01
    };

    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = covariance[i];
    }

    while (ros::ok()) {
        myBaseSensorData.encoder = encoder;
        odometry_cal();
        myBaseSensorData.encoder_old = myBaseSensorData.encoder;

        //odom_oriention trans to odom_quat
        odom_quat = tf::createQuaternionMsgFromYaw(myOdomCaculateData.oriention); //yaw trans quat

        //pub tf(odom->base_footprint)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = odom_child_frame_id;
        odom_trans.transform.translation.x = myOdomCaculateData.position_x;
        odom_trans.transform.translation.y = myOdomCaculateData.position_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //pub odom
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame_id;
        odom.child_frame_id = odom_child_frame_id;
        odom.pose.pose.position.x = myOdomCaculateData.position_x;
        odom.pose.pose.position.y = myOdomCaculateData.position_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = myOdomCaculateData.velocity_linear;
        odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.z = myOdomCaculateData.velocity_angular;
        odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0;
        odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);

        loop_rate.sleep();
        ros::spinOnce();
    }
    motor_speed = 0;
    return 0;
}
