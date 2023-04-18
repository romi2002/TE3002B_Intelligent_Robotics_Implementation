#include "characterization_node.hpp"
#include <functional>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

class GazeboCharacterizationNode : protected CharacterizationNode {
public:
    GazeboCharacterizationNode(const std::vector<CharacterizationConfig> &configs) :
     CharacterizationNode(configs),
     nh{} {
        resetClient = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    }

protected:
    void resetSim() override {
        ROS_INFO("Resetting sim");
        std_srvs::Empty srv;
        resetClient.call(srv);
    }
private:
    ros::NodeHandle nh;
    ros::ServiceClient resetClient;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "gazebo_characterization");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    auto move_forward = [&cmd_vel_pub]{
        geometry_msgs::Twist twist;
        twist.linear.x = 1;
        twist.angular.z = 0;
        cmd_vel_pub.publish(twist);
    };

    auto turn = [&cmd_vel_pub]{
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.angular.z = 1;
        cmd_vel_pub.publish(twist);
    };

    std::vector<CharacterizationConfig> configs;
    CharacterizationConfig config;
    config.cmdFunc = move_forward;
    config.prefixPath = "gazebo_move_forward_";
    configs.emplace_back(config);
    config.cmdFunc = turn;
    config.prefixPath = "gazebo_turn_";
    configs.emplace_back(config);

    GazeboCharacterizationNode gazeboNode(configs);
    ros::spin();
    return 0;
}