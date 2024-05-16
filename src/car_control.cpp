#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class carControlNode : public rclcpp::Node {
    private:
        rclcpp::Publisher<Twist>::SharedPtr mPub; // subscriber
        rclcpp::Subscription<LaserScan>::SharedPtr mSub;// publlisher
        rclcpp::TimerBase::SharedPtr mTimer; // timer

        int endIndex;   // 라이다 데이터 배열의 마지막 인덱스
        int firstIndex; // 라이다 데이터 배열의 첫번째 인덱스

        Twist mTwistMsg;    // 보낼 메시지를 저장하는 변수
        LaserScan laserMsg; // 받은 메시지를 저장하는 변수
 
        int controlCyleMs; // 제어 주기를 저장하는 변수

    public:
        // 생성자 선언
        carControlNode() : Node("car_control_node") {
            // 노드 이름 설정
            RCLCPP_INFO(get_logger(),"Car control Node Created");
            // publisher 설정
            mPub = create_publisher<Twist>("/cmd_vel",10);
            //subscriber 설정
            mSub = create_subscription<LaserScan>(
                "/scan",
                10,
                std::bind(
                    &carControlNode::getLaserMsg,
                    this,
                    std::placeholders::_1
                )
            );

            // 제어 주기를 여기에서 결정한다. 제어 주기에 맞춰 메시지를 퍼블리시한다.
            controlCyleMs = 10;
            mTimer = this->create_wall_timer(
                std::chrono::milliseconds(controlCyleMs),
                std::bind(&carControlNode::pubTwistMsg, this)
                );

            //set consts
            endIndex = 1080;
            firstIndex = 0;
        }

    private:
        void getLaserMsg(const LaserScan::SharedPtr msg){
            laserMsg = *msg;
        }  

        void pubTwistMsg(){
            
            // 이곳에 core code 함수를 작성한다. 

            // publish한다.
            mPub->publish(mTwistMsg);
            RCLCPP_INFO(
                get_logger(), 
                "publish success![]" 
            );
        }

};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<carControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}