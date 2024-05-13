#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class carControlNode : public rclcpp::Node {
    private:
        rclcpp::Publisher<Twist>::SharedPtr mPub;
        rclcpp::Subscription<LaserScan>::SharedPtr mSub;
        Twist mTwistMsg;

        int midIndex;
        int endIndex;
        int firstIndex;

        float frontDistLim;

    public:
        carControlNode() : Node("car_control_node") {
            RCLCPP_INFO(get_logger(),"Car control Node Created");
            mPub = create_publisher<Twist>("/cmd_vel",10);
            mSub = create_subscription<LaserScan>(
                "/scan",
                10,
                std::bind(
                    &carControlNode::subCallback,
                    this,
                    std::placeholders::_1
                )
            );

            //set consts
            endIndex = 1080;
            firstIndex = 0;
            midIndex = (endIndex-firstIndex)/2;

            frontDistLim = 0.8;
        }


    private:
        void subCallback(const LaserScan::SharedPtr msg){
            auto forwardDistance = (msg->ranges)[midIndex];
    
            // 제한 거리 초과일 땐 전진 미만이면 정지.
            if (forwardDistance > frontDistLim) {
                moveRobot(forwardDistance);
            } else 
                stopRobot();
        }  

        void moveRobot(const float &forwardDistance){
            mTwistMsg.linear.x = 0.5;
            mTwistMsg.angular.z = 0;
            mPub->publish(mTwistMsg);

            RCLCPP_INFO(
                get_logger(), 
                "front Dist: %f", 
                forwardDistance
            );
        }

        void stopRobot(){
            mTwistMsg.linear.x = 0.0;
            mTwistMsg.angular.z = 0;
            mPub->publish(mTwistMsg);
        }


};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<carControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}