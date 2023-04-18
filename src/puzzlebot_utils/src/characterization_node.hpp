#pragma once

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>

struct CharacterizationData {
    double x, y, yaw;
    double wr, wl;
};

struct CharacterizationConfig {
    std::function<void()> cmdFunc;
    int numIts{0};
    std::string prefixPath{"characterization_"};
};

std::ostream& operator<<(std::ostream& os, const CharacterizationData& data) {
    os << data.x << "," << data.y << "," << data.yaw << "," << data.wr << "," << data.wl;
    return os;
}

class CharacterizationNode{
public:
    CharacterizationNode(const std::vector<CharacterizationConfig> &config) :
        nh{}
    {
        this->config = config;
        poseSub = nh.subscribe<geometry_msgs::PoseStamped>("pose_sim", 10, [this](const geometry_msgs::PoseStampedConstPtr &msg){
            this->currentPose = msg->pose;
        });

        leftWheelSub = nh.subscribe<std_msgs::Float64>("wl", 10, [this](const std_msgs::Float64ConstPtr &msg){
            this->leftWheel = msg->data;
        });

        rightWheelSub = nh.subscribe<std_msgs::Float64>("wr", 10, [this](const std_msgs::Float64ConstPtr &msg){
            this->rightWheel = msg->data;
        });

        timer = nh.createTimer(ros::Duration(0.01), &CharacterizationNode::update, this);
    }
protected:
    virtual void resetSim() = 0;

    void writeToFile(const std::string &filename){
        std::ofstream outputFile(filename);
        if(!outputFile.is_open()){
            ROS_ERROR("Could not open file %s for writing", filename.c_str());
            return;
        }

        for(const auto &d : dataBuffer){
            outputFile << d << std::endl;
        }
        outputFile.close();
        ROS_INFO("Saved to: %s", filename.c_str());
    }

    void update(const ros::TimerEvent &e){
        config[config_i].cmdFunc();

        CharacterizationData dataPoint;
        dataPoint.x = currentPose.position.x;
        dataPoint.y = currentPose.position.y;

        tf2::Quaternion quat;
        tf2::fromMsg(currentPose.orientation, quat);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, dataPoint.yaw);

        dataPoint.wl = leftWheel;
        dataPoint.wr = rightWheel;

        dataBuffer.emplace_back(dataPoint);

        if(dataBuffer.size() >= num_samples){
            // Flush data buffer to file
            writeToFile(config[config_i].prefixPath + std::to_string(config[config_i].numIts) + ".csv");
            config[config_i].numIts++;

            // Reset simulation
            resetSim();

            // Remove samples from buffer
            dataBuffer.clear();
        }

        if(config[config_i].numIts >= num_reps){
            config_i++;

            if(config_i >= config.size()){
                // Done! Stop running
                timer.stop();
                return;
            }    
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber poseSub;
    ros::Subscriber leftWheelSub, rightWheelSub;
    ros::Timer timer;

    geometry_msgs::Pose currentPose;
    double leftWheel{0}, rightWheel{0};
    std::vector<CharacterizationData> dataBuffer;

    std::vector<CharacterizationConfig> config;
    int config_i{0};
    
    static constexpr int num_samples = 100;
    static constexpr int num_reps = 30;
};