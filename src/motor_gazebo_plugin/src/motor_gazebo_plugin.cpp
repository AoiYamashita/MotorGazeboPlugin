#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motor_gazebo_plugin/motor_gazebo_plugin.hpp"     // Header file.

#include <memory>

namespace gazebo{
    class MotorGazeboPluginPrivate{
        public:
            // ROS node for communication, managed by gazebo_ros.
            gazebo_ros::Node::SharedPtr ros_node_;

            // The joint that controls the movement of the belt:
            gazebo::physics::JointPtr motor_joint_;

            // Additional parametres:
            double motor_velocity_;
            double max_velocity_;
            
            // PUBLISH ConveyorBelt status:
            void PublishStatus();
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr status_pub_;      // Publisher.
            std_msgs::msg::Float32 status_msg_;                                    // motor status.

            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr msg_sub_;      // Subscriber.

            // WORLD UPDATE event:
            void OnUpdate();
            rclcpp::Time last_publish_time_;
            int update_ns_;
            gazebo::event::ConnectionPtr update_connection_;  // Connection to world update event. Callback is called while this is alive.
    };
    
    MotorGazeboPlugin::MotorGazeboPlugin()
    : impl_(std::make_unique<MotorGazeboPluginPrivate>())
    {
    }

    MotorGazeboPlugin::~MotorGazeboPlugin()
    {
    }

    void MotorGazeboPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

        impl_->motor_joint_ = _model->GetJoint("motor_joint");

        if(!impl_->motor_joint_){
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Motor joint not found, unable to start Motor plugin.");
            return;
        }

        impl_->max_velocity_ = _sdf->GetElement("max_velocity")->Get<double>();

        impl_->status_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>("/motor/rpm", 10);
        impl_->status_msg_.data = 0;

        impl_->msg_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>("/motor/speed",10,std::bind(&MotorGazeboPlugin::SubCallBack,this,std::placeholders::_1));

        double publish_rate = _sdf->GetElement("publish_rate")->Get<double>();
        impl_->update_ns_ = int((1/publish_rate) * 1e9);

        impl_->last_publish_time_ = impl_->ros_node_->get_clock()->now();

        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MotorGazeboPluginPrivate::OnUpdate, impl_.get()));

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO ConveyorBelt plugin loaded successfully.");
    }

    void MotorGazeboPluginPrivate::OnUpdate(){
        motor_joint_->SetVelocity(0, motor_velocity_);

        double motor_position = motor_joint_->Position(0);

        if (motor_position >= 100){
            motor_joint_->SetPosition(0, 0);
        }

        // Publish status at rate
        rclcpp::Time now = ros_node_->get_clock()->now();
        if (now - last_publish_time_ >= rclcpp::Duration(0, update_ns_)) {
            PublishStatus();
            last_publish_time_ = now;
        }
    }

    void MotorGazeboPlugin::SubCallBack(const std_msgs::msg::Float32::SharedPtr msg_) const
    {
        impl_->motor_velocity_ = (double)msg_->data;
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "I heard.\n");
    }

    void MotorGazeboPluginPrivate::PublishStatus(){

        status_pub_->publish(status_msg_);
    }
    GZ_REGISTER_MODEL_PLUGIN(MotorGazeboPlugin)
}
