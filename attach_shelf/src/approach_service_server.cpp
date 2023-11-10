#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <climits>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <utility>
#include <vector>
#include <algorithm>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

class Approach_Server : public rclcpp::Node{
    
    public:
        Approach_Server(): Node("approach_server"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){
            
            callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            rclcpp::SubscriptionOptions subscription_options;
            subscription_options.callback_group = callback_group_;

            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Approach_Server::scan_callback, this, std::placeholders::_1), subscription_options);

            srv_ = this->create_service<custom_interfaces::srv::GoToLoading>("/approach_shelf",
            std::bind(&Approach_Server::attach_callback, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,
            callback_group_);

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
            pub_elevator = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

            static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            timer_ = create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Approach_Server::publishTransform, this),callback_group_);


        }
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

        geometry_msgs::msg::Twist vel;
        std_msgs::msg::String ele;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr srv_;
        // tf2_ros::TransformBroadcaster broadcaster_{this}; //this will disappear in short time

        tf2_ros::StaticTransformBroadcaster static_broadcaster_{this};
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_elevator;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        float x1, x2, y1, y2;
        size_t accept_idx_size=0;

        bool ud_date_cart_frame = false;

        void publishTransform()
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.frame_id = "robot_base_link";
            transform_stamped.child_frame_id = "base_link";
            transform_stamped.transform.translation.x = 0.0;
            transform_stamped.transform.translation.y = 0.0;
            transform_stamped.transform.translation.z = 0.0;
            transform_stamped.transform.rotation.x = 0.0;
            transform_stamped.transform.rotation.y = 0.0;
            transform_stamped.transform.rotation.z = 0.0;
            transform_stamped.transform.rotation.w = 1.0;
            transform_stamped.header.stamp = this->now();

            static_transform_broadcaster_->sendTransform(transform_stamped);
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            
            // RCLCPP_INFO(this->get_logger(), "Intensity size:%zu" ,msg->intensities.size());
            // for(size_t i=0; i<msg->intensities.size();i++)
            // if(msg->intensities[i]>= 8000)
            // RCLCPP_INFO(this->get_logger(), "Scan intensity:%zu, %f",i, msg->intensities[i]);

            int minimum_idx_difference = 30;
            std::vector<std::pair<float, int>> intensity_cont;

            for(size_t i=0; i<msg->intensities.size();i++) 
            intensity_cont.emplace_back(msg->intensities[i],i);

            std::sort(intensity_cont.begin(), intensity_cont.end(),
                  [](const std::pair<float, int>& a, const std::pair<float, int>& b)-> 
                  bool {
                    return a.first > b.first;
                  });
            std::vector<int> accept_idx;
            for(const auto & pair : intensity_cont){
                

                bool close = false;
                for(const int acp : accept_idx){
                    if(std::abs(acp - pair.second) < minimum_idx_difference){
                        close = true;
                        break;
                    }
                }

                if(!close) accept_idx.push_back(pair.second);
                if(accept_idx.size() == 2) break;
            }
            accept_idx_size = accept_idx.size();

            int idx_1 = accept_idx[0];
            int idx_2 = accept_idx[1];
            
            float angle1 = msg->angle_min + idx_1*msg->angle_increment;
            float angle2 = msg->angle_min + idx_2*msg->angle_increment;

            x1 = msg->ranges[idx_1]* cos(angle1);
            y1 = msg->ranges[idx_1]* sin(angle1);

            x2 = msg->ranges[idx_2]* cos(angle2);
            y2 = msg->ranges[idx_2]* sin(angle2);
            

        }

        void set_map_cart_tf(){
                
            if (ud_date_cart_frame ) {

                RCLCPP_INFO(this->get_logger(), "X1 = %f",x1);
                RCLCPP_INFO(this->get_logger(), "Y1 = %f",y1);
                RCLCPP_INFO(this->get_logger(), "X2 = %f",x2);
                RCLCPP_INFO(this->get_logger(), "Y2 = %f",y2);

                static_broadcaster_.sendTransform(broadcast_transform(std::fabs((y1-y2)*0.95/2),0 ) );

                ud_date_cart_frame = false;
            }
        
        }

        void publish_cart_front_link(){

            geometry_msgs::msg::TransformStamped transform;
            geometry_msgs::msg::TransformStamped transform_stamped;

            try
            {
                transform = tf_buffer_.lookupTransform("robot_odom", "robot_cart_laser", rclcpp::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return;
            }
            transform_stamped = transform;
            transform_stamped.header.frame_id = "robot_odom";
            transform_stamped.child_frame_id = "cart_front_link";
            transform_stamped.header.stamp = this->now();

            static_transform_broadcaster_->sendTransform(transform_stamped);
        }

        void attach_callback(const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> req, 
        const std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> res){
            rclcpp::Rate l(1);
            publish_cart_front_link();
            if(req->attach_to_shelf == true && accept_idx_size == 2){
                ud_date_cart_frame = true;
                set_map_cart_tf();
                for(int i = 0 ; i< 2 ;i++) {
                    l.sleep();
                 }      
                res->length = std::sqrt( std::pow(x1-x2,2) + std::pow(y1-y2,2 ));
                move_towards_cart_frame();
                RCLCPP_INFO(this->get_logger(), "Approach call");

                RCLCPP_INFO(this->get_logger(), "Accept_idz = %zu",accept_idx_size);
                RCLCPP_INFO(this->get_logger(), "Length = %f", res->length);
                res->complete = true;
            }else if(req->attach_to_shelf == false && accept_idx_size == 2){
                ud_date_cart_frame = true;
                set_map_cart_tf();
                res->length = std::sqrt( std::pow(x1-x2,2) + std::pow(y1-y2,2 ));
                l.sleep();
                vel.linear.x = 0;
                vel.angular.z = 0;
                pub_->publish(vel);
                RCLCPP_INFO(this->get_logger(), "Approach not call but publish frame");
                res->complete = false;
            }else {
                RCLCPP_INFO(this->get_logger(), "Approach did not meet two legs");
                res->complete = false;
            }
        }

        geometry_msgs::msg::TransformStamped broadcast_transform( float x,float y){
                
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->now();
                t.header.frame_id = "cart_front_link";
                t.child_frame_id = "cart_frame";
                t.transform.translation.x = x ; //-x
                t.transform.translation.y = y;
                // t.transform.translation.z = 0.0; 
                // t.transform.rotation.x = 0.0;
                // t.transform.rotation.y = 0.0;
                // t.transform.rotation.z = 0;
                // t.transform.rotation.w = 1;

                return t;
        }

         void move_towards_cart_frame(){
            // Get the transform from base_link to cart_frame
            geometry_msgs::msg::TransformStamped transform;
            bool move_forward = true;

            try
            {
                transform = tf_buffer_.lookupTransform("cart_frame", "robot_base_link", rclcpp::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return;
            }

            float distance_to_point = std::sqrt(
                transform.transform.translation.x * transform.transform.translation.x +
                transform.transform.translation.y * transform.transform.translation.y);

            vel.linear.x = 0.15; 
            RCLCPP_INFO(this->get_logger(), "vel.linear = %f",vel.linear.x);
            // pub_->publish(vel);

            rclcpp::Rate l(5);
            rclcpp::Rate r(1);
            while (move_forward) {

                l.sleep();
                try
                {
                transform = tf_buffer_.lookupTransform("cart_frame", "robot_base_link", rclcpp::Time(0));
                distance_to_point = std::sqrt(
                    transform.transform.translation.x * transform.transform.translation.x +
                    transform.transform.translation.y * transform.transform.translation.y);

                if ( std::fabs(transform.transform.translation.x) < 0.05)
                {   
             
                    vel.linear.x = 0.0;
                    RCLCPP_INFO(this->get_logger(), "< TF -- X= %f",transform.transform.translation.x);

                    RCLCPP_INFO(this->get_logger(), "< TF -- Y= %f",transform.transform.translation.y);
                    RCLCPP_INFO(this->get_logger(), "< vel.linear = %f",vel.linear.x);
                    pub_->publish(vel);
                    pub_elevator->publish(ele);
                    for(int i = 0 ; i< 10 ;i++) {
                        r.sleep();
                        RCLCPP_INFO(this->get_logger(), "COUNT %i",i);
                    }     
                    vel.linear.x = -0.12;
                    for(int i = 0 ; i< 4 ;i++) {
                        r.sleep();
                        pub_->publish(vel);
                        RCLCPP_INFO(this->get_logger(), "COUNT %i",i);
                    }
                    vel.linear.x = 0.00;
                    pub_->publish(vel); 
                    move_forward = false;
                }else {
                    vel.linear.x = std::fabs(transform.transform.translation.x)*0.5 > 0.15 ? 0.12 : std::fabs(transform.transform.translation.x)*0.5; // move in x-direction of robot's frame
                    RCLCPP_INFO(this->get_logger(), "vel.linear = %f",vel.linear.x);
                    
                    RCLCPP_INFO(this->get_logger(), "TF -- X= %f",transform.transform.translation.x);

                    RCLCPP_INFO(this->get_logger(), "TF -- Y= %f",transform.transform.translation.y);
                    pub_->publish(vel);
                    
                }
                }
                catch (tf2::TransformException &ex)
                {
                    // Handle the exception (e.g., transform not available)
                    RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
                    // Stop moving the robot
                    vel.linear.x = 0.0;
                    pub_->publish(vel);
                    move_forward = false;
                }
            }
        }

};




int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Approach_Server>();
    executor.add_node(node);
    executor.spin();


    rclcpp::shutdown();
    return 0;
}