#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

class Attach_self : public rclcpp::Node{

    public:
        Attach_self(): Node("attach_self"){
            

            this->declare_parameter("obstacle", 0.0);
            this->declare_parameter<int>("degrees", 0);

            getting_params();

            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Attach_self::scan_callback, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            std::bind(&Attach_self::odom_callback, this, std::placeholders::_1));

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        }

    private:
        float obstacle;
        int degrees;
        geometry_msgs::msg::Twist vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        bool turn;  
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "obstacle: %f", obstacle);
            RCLCPP_INFO(this->get_logger(), "degrees: %i", degrees );
            if(msg->ranges[359] > obstacle && !turn ){
                vel.linear.x = 0.5;
            }else if (msg->ranges[359] < obstacle){
                vel.linear.x = 0;
                turn = true;
            }

            RCLCPP_INFO(this->get_logger(), "vel.linear: %f", vel.linear.x);

        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            float cur_degree = euler_degree_transform(msg)/M_PI*180;
            RCLCPP_INFO(this->get_logger(), "cur_degrees: %f", cur_degree );
            if (std::fabs(degrees - cur_degree) > 0.1 && turn ) {
                vel.angular.z = (degrees - cur_degree)/std::fabs(degrees - cur_degree)*0.4;
                RCLCPP_INFO(this->get_logger(), "vel.angular: %f", vel.angular.z);
                pub_->publish(vel);
            }else{
                vel.angular.z = 0;
                pub_->publish(vel);
            }
        }

        void getting_params(){
            obstacle = this->get_parameter("obstacle").get_parameter_value().get<float>();
            degrees = this->get_parameter("degrees").as_int();
        }

        float euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr msg){
                float x = msg->pose.pose.orientation.x;
                float y = msg->pose.pose.orientation.y;
                float z = msg->pose.pose.orientation.z;
                float w = msg->pose.pose.orientation.w; 

            return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Attach_self>());
    rclcpp::shutdown();
    return 0;
}