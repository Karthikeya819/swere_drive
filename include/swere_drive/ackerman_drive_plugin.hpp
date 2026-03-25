#ifndef ACKERMAN_DRIVE_PLUGIN_HPP
#define ACKERMAN_DRIVE_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>

namespace gazebo{
    class AckermanDrivePluginPrivate;
    class AckermanDrivePlugin : public gazebo::ModelPlugin{
    public:
        AckermanDrivePlugin();

        virtual ~AckermanDrivePlugin();

        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
        physics::JointPtr InitializeStepperJoint(std::string name, float stepper_p, float stepper_i, float stepper_d);
        void SetStepperJointPosition(float position1, float position2, float position3, float position4);
        physics::JointPtr InitializeWheelJoint(std::string name, float wheel_p, float wheel_i, float wheel_d);
        void SetWheelJointVelocity(float velocity1, float velocity2, float velocity3, float velocity4, float velocity5, float velocity6);
        void OnCmdVelReceived(const std::shared_ptr<const geometry_msgs::msg::Twist> msg);
        void publishOdom();

    private:
        std::unique_ptr<AckermanDrivePluginPrivate> impl_;
        physics::ModelPtr model;
        std::map<std::string, physics::JointPtr> stepperJointParams = {{"front_left_stepper_joint", NULL},{"front_right_stepper_joint", NULL},{"back_left_stepper_joint", NULL},{"back_right_stepper_joint", NULL},};
        std::map<std::string, physics::JointPtr> wheelJointParams = {{"front_left_wheel_joint", NULL},{"front_right_wheel_joint", NULL},{"back_left_wheel_joint", NULL},{"back_right_wheel_joint", NULL}};
        std::map<std::string, physics::JointPtr> wheelJointParams1 = {{"middle_left_wheel_joint", NULL},{"middle_right_wheel_joint", NULL},};
        std::map<std::string, std::string> generalParams = {{"updateRate","20"},{"commandTopic","cmd_vel"},{"odometryTopic","odom"},{"odometryFrame","odom"},{"robotBaseFrame",""},{"sixWheel","false"}};
        std::map<std::string, std::string> tuneParams = {{"wheelP","0.5"},{"wheelI","0.01"},{"wheelD","0.02"},{"stepperP","1000"},{"stepperI","0.0"},{"stepperD","50.0"}};
        std::map<std::string, float> robotParams = {{"wheelRadius",0.0},{"wheelSeperation",0.0},{"wheelBase",0.0}};
        rclcpp::Node::SharedPtr rosNode_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
        std::thread rosThread_; 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Odompublisher_;
        rclcpp::TimerBase::SharedPtr odom_timer_;
    };

}

#endif