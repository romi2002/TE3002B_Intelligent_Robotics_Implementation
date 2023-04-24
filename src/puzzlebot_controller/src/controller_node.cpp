#include <algorithm>

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

class PathController {
 public:
  PathController() : nh{} {
    // Setup publisher / subscribers
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    current_pose_pub = nh.advertise<geometry_msgs::Pose2D>("current_pose", 10);
    leftSub = nh.subscribe<std_msgs::Float32>(
        "wl", 10, [this](const std_msgs::Float32ConstPtr &msg) {
          this->leftWheel = msg->data;
        });
    rightSub = nh.subscribe<std_msgs::Float32>(
        "wr", 10, [this](const std_msgs::Float32ConstPtr &msg) {
          this->rightWheel = msg->data;
        });

    targetSub = nh.subscribe<geometry_msgs::Pose2D>(
        "target_pose", 10, [this](const geometry_msgs::Pose2DConstPtr &msg) {
          this->target_x = msg->x;
          this->target_y = msg->y;
        });

    last_time = ros::Time::now();

    // Setup robot constant params
    nh.param("l", l, 0.18); // Wheel base
    nh.param("r", r, 0.05); // Wheel Radius

    // Controller constants
    nh.param("max_u", max_u, 0.6);
    nh.param("max_r", max_r, 1.25);
    nh.param("k_u", k_u, 0.25);
    nh.param("k_r", k_r, 0.85);

    timer = nh.createTimer(ros::Duration(0.01), &PathController::update, this);
    timer.start();
  }

 protected:
  void sendVelocity(double u, double r) {
    // Clamp velocities to max
    if (std::abs(u) > max_u) u = std::copysign(max_u, u);
    if (std::abs(r) > max_r) r = std::copysign(max_r, r);

    geometry_msgs::Twist msg;
    msg.linear.x = u;
    msg.angular.z = r;
    last_u = u;
    last_r = r;
    cmd_vel_pub.publish(msg);
  }

  double wrapAngle(double angle) {
    // Wraps an angle to [-pi, pi]
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void updateEstimates(double dt) {
    double delta_u, delta_psi;
    // Closed loop -> Updates positions based on left and right wheel measurements
    if (closedLoop) {
      delta_u = r * (leftWheel + rightWheel) / 2.0 * dt;
      delta_psi = r * (leftWheel - rightWheel) / l * dt;
    } else {
      // Open loop -> Updates estimated position based on setpoints
      delta_u = last_u * dt;
      delta_psi = last_r * dt;
    }

    epsi += delta_psi;
    ex += std::cos(epsi) * delta_u;

    auto pose = geometry_msgs::Pose2D();
    pose.x = ex;
    pose.y = ey;
    current_pose_pub.publish(pose);
  }

  void update(const ros::TimerEvent &e) {
    double dt = ros::Time::now().toSec() - last_time.toSec();

    updateEstimates(0.01);

    // Compute error distance and angle to target
    double error_x = target_x - ex;
    double error_y = target_y - ey;
    double distance = std::hypot(error_x, error_y);
    double angle = wrapAngle(std::atan2(error_y, error_x) - epsi);

    // Compute velocities with proportional control
    double u = distance * std::cos(angle) * k_u + dist_integral * ki_u;
    double r = -angle * k_r - angle_integral * ki_r;
    ROS_INFO("%f %f", -angle * k_r, angle_integral * ki_u);

    // Stop when close
    if (distance < 0.1) {
      u = 0;
      r = 0;
    }

    dist_integral += distance * std::cos(angle) * dt;
    angle_integral += angle * dt;
    sendVelocity(u, r);

    last_time = ros::Time::now();
  }

 private:
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub;
  ros::Publisher current_pose_pub;
  ros::Subscriber leftSub, rightSub;
  ros::Subscriber targetSub;
  ros::Timer timer;

  // Estimated positions
  double ex{0}, ey{0}, epsi{0};
  double leftWheel{0}, rightWheel{0};
  double last_u{0}, last_r{0};
  ros::Time last_time;

  // TODO make these params
  double l{1}, r{1};
  double max_u{1}, max_r{1};

  double k_u{1}, k_r{1};
  double ki_u{0.0}, ki_r{0.0};

  double dist_integral{0}, angle_integral{0};

  double target_x{0}, target_y{0};
  bool closedLoop{true};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_controller");
  PathController controller;
  ros::spin();
  return 0;
}