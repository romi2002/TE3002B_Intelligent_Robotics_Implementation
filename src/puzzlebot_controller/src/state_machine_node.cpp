#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <puzzlebot_controller/MoveAction.h>

enum class StateMachineState {
    FollowLane,
    TurnLeft,
    TurnRight,
    Forward,
    Stop
};

class StateMachine{
public:
  StateMachine():
      nh_{},
      cmd_vel_pub_(nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
      traffic_light_sub_(nh_.subscribe("traffic_light",10,&StateMachine::traffic_light_cb,this)),
      traffic_signal_sub_(nh_.subscribe("yolo/prediction/class_name",10,&StateMachine::traffic_signal_cb,this)),
      lane_follower_sub_(nh_.subscribe("lane_follower/cmd_vel", 10, &StateMachine::lane_follower_cb_, this)),
      timer_(nh_.createTimer(ros::Duration(0.01), &StateMachine::mainLoop, this)),
      ac_("puzzlebot_move", true) {
      ROS_INFO("Starting State Machine");
      ROS_INFO("Waiting for puzzlebot action server");
      ac_.waitForServer();
      ROS_INFO("Found puzzlebot action server");
  }
protected:

  /**
  If no traffic light is detected, "None" should be published!
  */
  void traffic_light_cb(const std_msgs::String &msg){
    this->traffic_light_ = msg.data;
  }

  /**
  * If no traffic signal is detected, "None" should be published!
  */
  void traffic_signal_cb(const std_msgs::String &msg){
    this->traffic_signal_ = msg.data;
  }

  void lane_follower_cb_(const geometry_msgs::Twist &msg){
    lane_follower_cmd_vel_ = msg;
  }

  // TODO: Replace Messaages with actions
  void mainLoop(const ros::TimerEvent &e) {
    ROS_INFO("%d", state);
    if(traffic_signal_ != "None"){
        //Traffic signal was detected, take action
        if(traffic_signal_ == "Construction"){
            construction_detected_time = ros::Time::now();
        }

        //Stop signal was detected
        if(traffic_signal_ == "Stop"){
            state = StateMachineState::Stop;
        }

        //Turning signals
        if(traffic_signal_ == "left"){
            state = StateMachineState::TurnLeft;
        }

        if(traffic_signal_ == "right"){
            state = StateMachineState::TurnRight;
        }
    }


    //Check if last construction time < 5s, limit vel otherwise no limit
    if((ros::Time::now() - construction_detected_time).toSec() < 5 || traffic_light_ == "Yellow"){
        ROS_INFO("Limiting velocity, construction zone!");
        set_velocity = limited_velocity;
    } else {
        set_velocity = normal_velocity;
    }

    if(traffic_light_ == "Red"){
        state = StateMachineState::Stop;
    }

    // Main state machine loop
    if(state == StateMachineState::FollowLane){
        // Send lane follower cmd_vel to robot
        auto cmd = lane_follower_cmd_vel_;
        cmd.linear.x = set_velocity;
        cmd_vel_pub_.publish(cmd);
    } else if(state == StateMachineState::Stop){
        // Send 0 cmd_vel, do nothing
        geometry_msgs::Twist stop_cmd;
        cmd_vel_pub_.publish(stop_cmd);

        if(traffic_light_ != "Red" && traffic_signal_ != "Stop"){
            // Exit out of stopped state
            state = StateMachineState::FollowLane;
        }
    } else if(state == StateMachineState::TurnLeft){
        // Run left turn blocking action
        turnLeft();
        state = StateMachineState::FollowLane;
    } else if(state == StateMachineState::TurnRight){
        // Run right turn blocking action
        turnRight();
        state = StateMachineState::FollowLane;
    } else if(state == StateMachineState::TurnRight){
        // Move forward, cross intersection
        forward();
        state = StateMachineState::FollowLane;
    }
  }

  void turnLeft(){
    move_and_turn(-M_PI/2);
  }

  void turnRight(){
    move_and_turn(M_PI/2);
  }

  void move_and_turn(double angle, double dist = 0.2){
    ROS_INFO("Moving forward");
    puzzlebot_controller::MoveActionGoal action;
    action.goal.goal.x = dist;
    action.goal.goal.z = 0;
    ac_.sendGoal(action.goal);
    ac_.waitForResult(ros::Duration(60.0));
    ROS_INFO("Turning");
    action.goal.goal.x = 0.0;
    action.goal.goal.z = angle;
    ac_.sendGoal(action.goal);
    ac_.waitForResult(ros::Duration(60.0));
    ROS_INFO("Moving forward");
    action.goal.goal.x = dist;
    action.goal.goal.z = 0;
    ac_.sendGoal(action.goal);
    ac_.waitForResult(ros::Duration(60.0));
    ROS_INFO("Finished");
  }

  void forward(double dist = 0.4){
    ROS_INFO("Moving forward");
    puzzlebot_controller::MoveActionGoal action;
    action.goal.goal.x = dist;
    action.goal.goal.z = 0;
    ac_.sendGoal(action.goal);
    ac_.waitForResult(ros::Duration(60.0));
    ROS_INFO("Finished");
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber traffic_light_sub_;
  ros::Subscriber traffic_signal_sub_;
  ros::Subscriber lane_follower_sub_;
  actionlib::SimpleActionClient<puzzlebot_controller::MoveAction> ac_;
  
  geometry_msgs::Twist lane_follower_cmd_vel_;

  // ros::Publisher pose_pub_; 
  ros::Timer timer_;
  std::string traffic_signal_{""};
  std::string traffic_light_{""};
  StateMachineState state = StateMachineState::FollowLane;

  ros::Time construction_detected_time;
  double set_velocity{0};

  const double normal_velocity = 0.05;
  const double limited_velocity = 0.01;
};

int main(int argc, char** argv){
  ros::init(argc,argv,"state_machine_node");
  StateMachine node;
  ros::spin();
  return 0;
}