#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

class Odometry{
    public:
    Odometry():
        nh_{},
        pose_pub_(nh_.advertise<geometry_msgs::PoseStamped>("pose",10)),
        wl_sub_(nh_.subscribe("wl",10,&Odometry::wl_sub_cb,this)),
        wr_sub_(nh_.subscribe("wr",10,&Odometry::wr_sub_cb,this)),
        timer_(nh_.createTimer(ros::Duration(0.01), &Odometry::update_states, this)), // why timer
        last_time_{ros::Time::now()}
    {}
    protected:

    void wl_sub_cb(std_msgs::Float32 msg){
        wl_ = msg.data;
    }

    void wr_sub_cb(std_msgs::Float32 msg){
        wr_ = msg.data;
    }

    void update_states(const ros::TimerEvent &e) {
        ros::Time current_time = ros::Time::now();
        double dt = current_time.toSec() - last_time_.toSec();

        double delta_theta, delta_d;

        // find delta
        delta_theta = r_ * (wl_ - wr_) / l_ * dt;
        delta_d = r_ * (wl_ + wr_) / 2.0 * dt;

        // update x, y, theta 
        
        theta_+=delta_theta;

        x_ += delta_d*cos(theta_);
        y_ += delta_d*sin(theta_);

        ROS_INFO("%f, %f, %f, %f", dt, x_, y_, theta_);
        //publish msg
        auto pose_msg = geometry_msgs::PoseStamped();
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x_;
        pose_msg.pose.position.y = y_;
        pose_msg.pose.position.z = 0;

        auto q_rot = tf2::Quaternion();
        q_rot.setRPY(0,0,theta_);

        pose_msg.pose.orientation.w = q_rot.w();
        pose_msg.pose.orientation.x = q_rot.x();
        pose_msg.pose.orientation.y = q_rot.y();
        pose_msg.pose.orientation.z = q_rot.z();

        pose_pub_.publish(pose_msg);

        // Broadcast
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.child_frame_id = "puzzlebot";
        transformStamped.header.frame_id = "map";
        transformStamped.header.stamp = current_time;
        transformStamped.transform.translation.x = x_;
        transformStamped.transform.translation.y = y_;
        transformStamped.transform.translation.z = 0;

        transformStamped.transform.rotation.w = q_rot.w();
        transformStamped.transform.rotation.x = q_rot.x();
        transformStamped.transform.rotation.y = q_rot.y();
        transformStamped.transform.rotation.z = q_rot.z();

        br_.sendTransform(transformStamped);
        

        last_time_ = ros::Time::now();
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber wl_sub_;
    ros::Subscriber wr_sub_;
    
    ros::Publisher pose_pub_; 
    double wl_{0}, wr_{0};
    double x_{0}, y_{0}, theta_{0};
    double r_{0.05}, l_{0.18}; // radius, length
    ros::Time last_time_;
    ros::Timer timer_;
    // http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
    tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"odom_node");
    Odometry node;
    ros::spin();
    return 0;
}