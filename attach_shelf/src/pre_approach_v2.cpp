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
#include "custom_interfaces/srv/go_to_loading.hpp"


class Attach_self : public rclcpp::Node{

    public:
        Attach_self(int &argc, char **argv): Node("attach_self"){
            
            ob = argv[2];
            de = argv[4];
            fa = argv[6];

            getting_params();

            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Attach_self::scan_callback, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            std::bind(&Attach_self::odom_callback, this, std::placeholders::_1));

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

            client = this->create_client<custom_interfaces::srv::GoToLoading>(
            "/approach_shelf");      

            while (!client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
            }
        }

    

    private:
        float obstacle;
        int degrees;
        bool final_approach;
        bool send = true;   
        std::string ob;
        std::string de;
        std::string fa;
        rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client;
        
        geometry_msgs::msg::Twist vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        bool turn;  
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            // RCLCPP_INFO(this->get_logger(), "obstacle: %f", obstacle);
            // if (!final_approach) {
            //     RCLCPP_INFO(this->get_logger(), "final_approach: false");
            // }else {
            //     RCLCPP_INFO(this->get_logger(), "final_approach: true");
            // }
            // RCLCPP_INFO(this->get_logger(), "degrees: %i", degrees );
            
            if(msg->ranges[359] > obstacle && !turn && send ){
                vel.linear.x = 0.5;
            }else if (msg->ranges[359] < obstacle && send){
                vel.linear.x = 0;
                turn = true;
            }

            // RCLCPP_INFO(this->get_logger(), "vel.linear: %f", vel.linear.x);

        }
        

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            float cur_degree = euler_degree_transform(msg)/M_PI*180;
            // RCLCPP_INFO(this->get_logger(), "cur_degrees: %f", cur_degree );

            if (std::fabs(degrees - cur_degree) > 0.5 && turn && send) {
                vel.angular.z = (degrees - cur_degree)/std::fabs(degrees - cur_degree)*0.4;
                pub_->publish(vel);
            }else if(std::fabs(degrees - cur_degree) < 0.5 && turn && send){
                auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
                vel.angular.z = 0;
                pub_->publish(vel);
                request->attach_to_shelf = final_approach;
                client->async_send_request(request, std::bind(&Attach_self::service_response, this, std::placeholders::_1));
                send = false;
            }else if(send){
                vel.angular.z = 0;
                pub_->publish(vel);
            }
        }

        void getting_params(){
            obstacle = std::stof(ob);
            degrees = std::stoi(de);
            final_approach = fa == "false" ? false : true;
        }

        void service_response(rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture fut){
            auto status = fut.wait_for(std::chrono::seconds(1));
            if(status == std::future_status::ready){
                auto response = fut.get();
                RCLCPP_INFO(this->get_logger(), "Service was called");
                std::string fb = response->complete ? "true" : "false";
                RCLCPP_INFO(this->get_logger(), "Recevied final_approach: %s", fb.c_str());
            }
            else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            }
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
    rclcpp::spin(std::make_shared<Attach_self>(argc, argv));
    rclcpp::shutdown();
    return 0;
}