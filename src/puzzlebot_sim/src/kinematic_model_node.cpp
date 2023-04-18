#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "puzzlebot_sim/ResetPosition.h"

class KineamticModel{
public:
    KineamticModel() : 
        nh{},
        posePub(nh.advertise<geometry_msgs::PoseStamped>("pose_sim", 10)),
        leftPub(nh.advertise<std_msgs::Float64>("wl", 10)),
        rightPub(nh.advertise<std_msgs::Float64>("wr", 10))
    {
        cmd_vel.linear.x = 0; // u
        cmd_vel.angular.z = 0; // r

        currentPose.pose.position.x = 0;
        currentPose.pose.position.y = 0;
        currentPose.pose.position.z = 0;

        velSub = nh.subscribe("cmd_vel", 1, &KineamticModel::vel_cb, this);
        timer = nh.createTimer(ros::Duration(dt), &KineamticModel::update, this);
        timer.start();

        resetService = nh.advertiseService("reset_position", &KineamticModel::reset_position, this);
    }
protected:
    void vel_cb(const geometry_msgs::Twist &msg) {
        cmd_vel = msg;
    }

    void update(const ros::TimerEvent &e) {
        double u = cmd_vel.linear.x;
        double omega = cmd_vel.angular.z;

        // Update current pose
        yaw += omega * dt;
        currentPose.pose.position.x += u * std::cos(yaw) * dt;
        currentPose.pose.position.y += u * std::sin(yaw) * dt;
        ROS_INFO("x %f y %f yaw %f omega %f", currentPose.pose.position.x, currentPose.pose.position.y, yaw, omega);

        // Publish current pose
        tf2::Quaternion quat;
        quat.setRPY(0,0,yaw);
        currentPose.pose.orientation = tf2::toMsg(quat);

        currentPose.header.stamp = ros::Time::now();
        posePub.publish(currentPose);

        // Publish wheel velocities from cmd_vel
        std_msgs::Float64 leftMsg, rightMsg;
        leftMsg.data = (u - (l * omega) / 2) * (1.0 / r);
        rightMsg.data = (u + (l * omega) / 2) * (1.0 / r);

        leftPub.publish(leftMsg);
        rightPub.publish(rightMsg);
    }

    bool reset_position(
        puzzlebot_sim::ResetPosition::Request &req,
        puzzlebot_sim::ResetPosition::Response &res
    ){
        this->currentPose.pose = req.initialPose;
        ROS_INFO("Resetting position to x=%f, y=%f", req.initialPose.position.x , req.initialPose.position.y);
        return true;
    }
private:
    ros::NodeHandle nh;
    ros::Publisher posePub, leftPub, rightPub;
    ros::Subscriber velSub;
    ros::Timer timer;
    ros::ServiceServer resetService;

    geometry_msgs::PoseStamped currentPose;
    geometry_msgs::Twist cmd_vel;
    double yaw{0};

    // Simulation constants
    static constexpr double dt{0.01};
    static constexpr double r{1}; // wheel radius
    static constexpr double l{1}; // wheel base
};

int main(int argc, char **argv){
    ros::init(argc, argv, "puzzlebot_kinematic_model");
    KineamticModel node;
    ros::spin();
    return 0;    
}