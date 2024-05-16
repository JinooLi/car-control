#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class LaserInfo {   // 라이다 관련 상수를 저장하는 class
    public:
        int lastIndex;          // 라이다 데이터 배열의 마지막 인덱스
        int firstIndex;         // 라이다 데이터 배열의 첫번째 인덱스
        float angleMin;         // 라이다 최소 각도 [rad]
        float angleMax;         // 라이다 최대 각도 [rad]
        float angleIncrement;   // 라이다 측정 각간 각도
};

class carControlNode : public rclcpp::Node {
    private:
        rclcpp::Publisher<Twist>::SharedPtr publisher; // subscriber
        rclcpp::Subscription<LaserScan>::SharedPtr subscriber;// publlisher
        rclcpp::TimerBase::SharedPtr mTimer; // timer

        LaserInfo laserinfo; // 라이다 관련 상수를 저장하는 객체 변수

        Twist mTwistMsg;    // 보낼 메시지를 저장하는 변수
        LaserScan laserMsg; // 받은 메시지를 저장하는 변수
        bool isLaserMsgIn;  // 레이저 메시지가 한번이라도 들어왔으면 true, 아니면 false
 
        int pubCycleMs; // 제어 주기를 저장하는 변수

    public:
        // 생성자 선언
        carControlNode() : Node("car_control_node") {
            // 노드 이름 설정
            RCLCPP_INFO(get_logger(),"Car control Node Created");
            // 라이다 정보 객체 생성
            laserinfo = LaserInfo();
            // publisher 설정
            publisher = create_publisher<Twist>("/cmd_vel",10);
            // flag 초기화
            isLaserMsgIn = false;
            //subscriber 설정
            subscriber = create_subscription<LaserScan>(
                "/scan",
                10,
                std::bind(
                    &carControlNode::getLaserMsg,
                    this,
                    std::placeholders::_1
                )
            );

            // publsish 주기를 여기에서 결정한다. 
            pubCycleMs = 10;
            mTimer = this->create_wall_timer(
                std::chrono::milliseconds(pubCycleMs),
                std::bind(&carControlNode::pubTwistMsg, this)
            );
        }

    private:
        // 라이다 메시지를 기반으로 라이다 관련 상수를 설정하는 함수
        void setLaserConstInfo(){
            laserinfo.angleMax = laserMsg.angle_max;
            laserinfo.angleMin = laserMsg.angle_min;
            laserinfo.angleIncrement = laserMsg.angle_increment;
            laserinfo.lastIndex = (laserinfo.angleMax - laserinfo.angleMin)/laserinfo.angleIncrement;
            laserinfo.firstIndex = 0;
        }

        // 라이다 메시지를 받아온다.
        void getLaserMsg(const LaserScan::SharedPtr msg){
            laserMsg = *msg;
            if(!isLaserMsgIn){
                setLaserConstInfo();
                isLaserMsgIn = true;
            }
            RCLCPP_INFO(
                get_logger(), 
                "angle_min : %f, angle_max : %f",
                msg->angle_min, msg->angle_max
            );
        }  

        void pubTwistMsg(){
            // 레이저 정보가 들어오기 전까지 아무것도 하지 않는다.
            if(isLaserMsgIn){
                // 이곳에 core code 함수를 작성한다. 이 함수는 다음 입력과 출력을 가진다.
                // 입력 : laserMsg
                // 출력 : mTwistMsg
                mTwistMsg.angular.z = 0;
                mTwistMsg.linear.x = 0.7;

                // publish한다.
                publisher->publish(mTwistMsg);
                RCLCPP_INFO(
                    get_logger(), 
                    "publish success![]" 
                );
            }
        }

};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<carControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}