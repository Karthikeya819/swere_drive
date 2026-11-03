#include <swere_drive/ackerman_drive_plugin.hpp>
#include <cmath>

#include <memory>

#define RED     "\033[1;31m"
#define GREEN   "\033[1;32m"
#define BLUE    "\033[1;34m"
#define PURPLE  "\033[1;35m"
#define ORANGE  "\033[1;33m"
#define CYAN    "\033[1;36m"
#define RESET   "\033[0m"

namespace gazebo {

class AckermanDrivePluginPrivate{};
    AckermanDrivePlugin::AckermanDrivePlugin()
    : impl_(std::make_unique<AckermanDrivePluginPrivate>()) {
    std::cout << CYAN << "Initialized the Ackerman Drive Plugin" << RESET << std::endl;
    }

    AckermanDrivePlugin::~AckermanDrivePlugin() {
        updateConnection_.reset();
        if (this->rosThread_.joinable()){
            rclcpp::shutdown();
            this->rosThread_.join();
        }
        std::cout << RED << "Destroyed the Ackerman Drive Plugin" << RESET << std::endl;
    }

    void AckermanDrivePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {

        if (!rclcpp::ok()){
            rclcpp::init(0, nullptr);
        }
        this->rosNode_ = rclcpp::Node::make_shared("ackerman_drive_plugin_node");
        this->rosNode_->set_parameter(rclcpp::Parameter("use_sim_time", true));

        this->model = model;

        bool paramNotSpecified = false;

        // Get General Params from SDF
        for(auto& param : this->generalParams){
            const std::string& paramName = param.first;
            if (sdf->HasElement(paramName)) {
                param.second = sdf->Get<std::string>(paramName);
            }else if(param.second != ""){
                std::cout << ORANGE << "Parameter " << paramName << " not specified in plugin! Taking default Value as "<< param.second << RESET << std::endl;
            }else {
                paramNotSpecified = true;
                std::cout << RED << "Parameter " << paramName << " not specified in plugin!" << RESET << std::endl;
            }
        }

        if (!swere_drive::ParseBoolean(generalParams["sixWheel"], sixWheel_)) {
            RCLCPP_ERROR(rosNode_->get_logger(), "sixWheel must be true, false, 1, or 0");
            return;
        }

        // Get Tune Params from sdf
        for(auto& param : this->tuneParams){
            const std::string& paramName = param.first;
            if (sdf->HasElement(paramName)) {
                param.second = sdf->Get<std::string>(paramName);
            }else if(param.second != ""){
                std::cout << ORANGE << "Parameter " << paramName << " not specified in plugin! Taking default Value as "<< param.second << RESET << std::endl;
            }else {
                paramNotSpecified = true;
                std::cout << RED << "Parameter " << paramName << " not specified in plugin!" << RESET << std::endl;
            }
        }

        // Get Robot Params
        for(auto& param : this->robotParams){
            const std::string& paramName = param.first;
            if (sdf->HasElement(paramName)) {
                param.second = stof(sdf->Get<std::string>(paramName));
            }else if(param.second != 0.0){
                std::cout << ORANGE << "Parameter " << paramName << " not specified in plugin! Taking default Value as "<< param.second << RESET << std::endl;
            }else {
                paramNotSpecified = true;
                std::cout << RED << "Parameter " << paramName << " not specified in plugin!" << RESET << std::endl;
            }
        }

        // Get Stepper Params from SDF
        float stepperP = stof(this->tuneParams["stepperP"]), stepperI = stof(this->tuneParams["stepperI"]),stepperD = stof(this->tuneParams["stepperD"]);
        for(auto& param : this->stepperJointParams){
            const std::string& paramName = param.first;
            if (sdf->HasElement(paramName)) {
                std::string joint_name = sdf->Get<std::string>(paramName);
                param.second = InitializeStepperJoint(joint_name, stepperP, stepperI, stepperD);
                if (!param.second) paramNotSpecified = true;
            } else {
                paramNotSpecified = true;
                std::cout << RED << "Parameter " << paramName << " not specified in plugin!" << RESET << std::endl;
            }
        }
        // Get Wheel Params from SDF
        float wheelP = stof(this->tuneParams["wheelP"]), wheelI = stof(this->tuneParams["wheelI"]),wheelD = stof(this->tuneParams["wheelD"]);
        for(auto& param : this->wheelJointParams){
            const std::string& paramName = param.first;
            if (sdf->HasElement(paramName)) {
                std::string joint_name = sdf->Get<std::string>(paramName);
                param.second = InitializeWheelJoint(joint_name, wheelP, wheelI, wheelD);
                if (!param.second) paramNotSpecified = true;
            } else {
                paramNotSpecified = true;
                std::cout << RED << "Parameter " << paramName << " not specified in plugin!" << RESET << std::endl;
            }
        }
        if(sixWheel_){
            for(auto& param : this->wheelJointParams1){
                const std::string& paramName = param.first;
                if (sdf->HasElement(paramName)) {
                    std::string joint_name = sdf->Get<std::string>(paramName);
                    param.second = InitializeWheelJoint(joint_name, wheelP, wheelI, wheelD);
                    if (!param.second) paramNotSpecified = true;
                } else {
                    paramNotSpecified = true;
                    std::cout << RED << "Parameter " << paramName << " not specified in plugin!" << RESET << std::endl;
                }
            }
        }
        if(paramNotSpecified)return;
        for (const auto &param : robotParams) {
            if (!std::isfinite(param.second) || param.second <= 0) {
                RCLCPP_ERROR(rosNode_->get_logger(), "Robot dimensions must be finite and positive");
                return;
            }
        }
        const double updateRate = std::stod(generalParams["updateRate"]);
        if (!std::isfinite(updateRate) || updateRate <= 0 || updateRate > 1000) {
            RCLCPP_ERROR(rosNode_->get_logger(), "updateRate must be in (0, 1000] Hz");
            return;
        }
        this->cmdVelSub_ = this->rosNode_->create_subscription<geometry_msgs::msg::Twist>(this->generalParams["commandTopic"],10,std::bind(&AckermanDrivePlugin::OnCmdVelReceived, this, std::placeholders::_1));
        std::cout << GREEN << "Subscribed to " << generalParams["commandTopic"] << " Topic!" << RESET << std::endl;
        this->Odompublisher_ = this->rosNode_->create_publisher<nav_msgs::msg::Odometry>(this->generalParams["odometryTopic"], 10);
        auto timer_callback = std::bind(&AckermanDrivePlugin::publishOdom, this);
        this->odom_timer_ = this->rosNode_->create_wall_timer(std::chrono::duration<double>(1.0 / updateRate),timer_callback);

        SetStepperJointPosition(0,0,0,0);
        SetWheelJointVelocity(0,0,0,0,0,0);
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&AckermanDrivePlugin::OnUpdate, this));
        this->rosThread_ = std::thread([this]() { rclcpp::spin(this->rosNode_); });
    }

    physics::JointPtr AckermanDrivePlugin::InitializeStepperJoint(std::string name, float stepper_p, float stepper_i, float stepper_d){
        physics::JointPtr joint = this->model->GetJoint(name);
        if (!joint){
            std::cout << RED << "Joint not found!" << RESET << std::endl;
            return NULL;
        }
        // common::PID pid = common::PID(1000.0, 0.0, 50.0);
        common::PID pid = common::PID(stepper_p, stepper_i, stepper_d);
        this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
        this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), 0);
        return joint;
    }
    void AckermanDrivePlugin::SetStepperJointPosition(float position1, float position2, float position3, float position4){
        // Order front_left front_right back_left back_right
        this->model->GetJointController()->SetPositionTarget(this->stepperJointParams["front_left_stepper_joint"]->GetScopedName(),position1);
        this->model->GetJointController()->SetPositionTarget(this->stepperJointParams["front_right_stepper_joint"]->GetScopedName(),position2);
        this->model->GetJointController()->SetPositionTarget(this->stepperJointParams["back_left_stepper_joint"]->GetScopedName(),position3);
        this->model->GetJointController()->SetPositionTarget(this->stepperJointParams["back_right_stepper_joint"]->GetScopedName(),position4);
    }
    physics::JointPtr AckermanDrivePlugin::InitializeWheelJoint(std::string name, float wheel_p, float wheel_i, float wheel_d){
        physics::JointPtr joint = this->model->GetJoint(name);
        if (!joint){
            std::cout << RED << name << "Joint not found!" << RESET << std::endl;
            return NULL;
        }
        //common::PID pid = common::PID(0.5, 0.01, 0.02);
        common::PID pid = common::PID(wheel_p, wheel_i, wheel_d);
        this->model->GetJointController()->SetVelocityPID(joint->GetScopedName(), pid);
        return joint;
    }
    void AckermanDrivePlugin::SetWheelJointVelocity(float velocity1, float velocity2, float velocity3, float velocity4, float velocity5, float velocity6){
        // velocity is in rad/s
        this->model->GetJointController()->SetVelocityTarget(this->wheelJointParams["back_left_wheel_joint"]->GetScopedName(), velocity5);
        this->model->GetJointController()->SetVelocityTarget(this->wheelJointParams["back_right_wheel_joint"]->GetScopedName(), velocity6);
        if(sixWheel_){
            this->model->GetJointController()->SetVelocityTarget(this->wheelJointParams1["middle_left_wheel_joint"]->GetScopedName(), velocity3);
            this->model->GetJointController()->SetVelocityTarget(this->wheelJointParams1["middle_right_wheel_joint"]->GetScopedName(), velocity4);
        }
        this->model->GetJointController()->SetVelocityTarget(this->wheelJointParams["front_left_wheel_joint"]->GetScopedName(), velocity1);
        this->model->GetJointController()->SetVelocityTarget(this->wheelJointParams["front_right_wheel_joint"]->GetScopedName(), velocity2);
    }
    void AckermanDrivePlugin::OnCmdVelReceived(const std::shared_ptr<const geometry_msgs::msg::Twist> msg){
        std::lock_guard<std::mutex> lock(commandMutex_);
        float linear_x = msg->linear.x;
        float angular_z = msg->angular.z;
        float linear_y = msg->linear.y;

        // Constants
        const float wheel_radius = robotParams["wheelRadius"]; // Radius of wheels in meters
        const float wheel_separation = robotParams["wheelSeperation"]; // Distance between left and right wheels in meters
        const float wheel_base = robotParams["wheelBase"]; // Distance between front and rear axles in meters

        if (!std::isfinite(linear_x) || !std::isfinite(linear_y) || !std::isfinite(angular_z)) {
            crabTransition_.Cancel();
            SetWheelJointVelocity(0,0,0,0,0,0);
            return;
        }
        const bool crab = linear_y != 0 && linear_x == 0 && angular_z == 0 && !sixWheel_;
        if (!crab) crabTransition_.Cancel();
        if(crab){
            float rads = linear_y / wheel_radius; // here v = wr
            SetStepperJointPosition(1.57, 1.57, 1.57, 1.57);
            crabTransition_.Begin(model->GetWorld()->SimTime().Double(), rads);
            if (crabTransition_.Ready(model->GetWorld()->SimTime().Double())) {
                SetWheelJointVelocity(rads,rads,0,0,rads,rads);
            } else {
                SetWheelJointVelocity(0,0,0,0,0,0);
            }
        }else if(msg->angular.z == 0){
            float rads = msg->linear.x / wheel_radius; // Radius here v = wr
            SetStepperJointPosition(0,0,0,0);
            SetWheelJointVelocity(rads, rads, rads, rads, rads, rads);
        }else if(msg->linear.x == 0){
            float ms = (msg->angular.z*wheel_separation)/2; // wheel_sep here v = wl/2
            float rads = ms / wheel_radius; // Radius here v = wr
            SetStepperJointPosition(-0.785,0.785,0.785,-0.785);
            SetWheelJointVelocity(-rads, rads, 0, 0, -rads, rads);
        }else{
            float R_icr = linear_x / angular_z;
            float R_front_left = R_icr - (wheel_separation / 2);
            float R_front_right = R_icr + (wheel_separation / 2);
            float R_back_left = R_icr - (wheel_separation / 2);
            float R_back_right = R_icr + (wheel_separation / 2);

            // Compute steering angles
            float position_front_left = atan(wheel_base / R_front_left);
            float position_front_right = atan(wheel_base / R_front_right);
            float position_back_left = -atan(wheel_base / R_back_left);
            float position_back_right = -atan(wheel_base / R_back_right);
            //std::cout << "front_left:" << position_front_left << " front_right:" << position_front_right << " back_left:" << position_back_left << " back_right:" << position_back_left << std::endl;

            // Compute linear velocity for each wheel
            float v_front_left = linear_x * (R_front_left / R_icr);
            float v_front_right = linear_x * (R_front_right / R_icr);
            float v_back_left = linear_x * (R_back_left / R_icr);
            float v_back_right = linear_x * (R_back_right / R_icr);

            // Convert linear velocities to angular velocities (rad/s)
            float rads_front_left = v_front_left / wheel_radius;
            float rads_front_right = v_front_right / wheel_radius;
            float rads_back_left = v_back_left / wheel_radius;
            float rads_back_right = v_back_right / wheel_radius;
            float rads_middle_left = linear_x / wheel_radius;
            float rads_middle_right = linear_x / wheel_radius;

            // Set steering angles and wheel velocities
            SetStepperJointPosition(position_front_left, position_front_right, position_back_left, position_back_right);
            SetWheelJointVelocity(rads_front_left, rads_front_right, rads_middle_left, rads_middle_right, rads_back_left, rads_back_right);
        }

    }
    void AckermanDrivePlugin::OnUpdate(){
        std::lock_guard<std::mutex> lock(commandMutex_);
        if (crabTransition_.Ready(model->GetWorld()->SimTime().Double())) {
            const float rads = crabTransition_.Speed();
            SetWheelJointVelocity(rads,rads,0,0,rads,rads);
        }
    }

    void AckermanDrivePlugin::publishOdom(){
        // Has to Implement custom Reverse Kinematics, for time being I'm using gazebo co-ordinates
        ignition::math::Pose3d pose = this->model->WorldPose();
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = this->rosNode_->get_clock()->now();
        odom_msg.header.frame_id = generalParams["odometryFrame"];
        odom_msg.child_frame_id = generalParams["robotBaseFrame"];

        odom_msg.pose.pose.position.x = pose.Pos().X();
        odom_msg.pose.pose.position.y = pose.Pos().Y();
        odom_msg.pose.pose.position.z = pose.Pos().Z();

        ignition::math::Quaterniond quat = pose.Rot();
        odom_msg.pose.pose.orientation.x = quat.X();
        odom_msg.pose.pose.orientation.y = quat.Y();
        odom_msg.pose.pose.orientation.z = quat.Z();
        odom_msg.pose.pose.orientation.w = quat.W();

        ignition::math::Vector3d linearVel = this->model->WorldLinearVel();
        odom_msg.twist.twist.linear.x = linearVel.X();
        odom_msg.twist.twist.linear.y = linearVel.Y();
        odom_msg.twist.twist.linear.z = linearVel.Z();

        ignition::math::Vector3d angularVel = this->model->WorldAngularVel();
        odom_msg.twist.twist.angular.x = angularVel.X();
        odom_msg.twist.twist.angular.y = angularVel.Y();
        odom_msg.twist.twist.angular.z = angularVel.Z();

        this->Odompublisher_->publish(odom_msg);

    }

    GZ_REGISTER_MODEL_PLUGIN(AckermanDrivePlugin);

}