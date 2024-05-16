#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "../include/car_control/core.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;



class carControlNode : public rclcpp::Node {
private:
    rclcpp::Publisher<Twist>::SharedPtr twistPub; // twist publisher
    rclcpp::Subscription<LaserScan>::SharedPtr laserSub; // laser subscriber
    rclcpp::TimerBase::SharedPtr pubTimer; // timer for publish cycle

    core::LaserInfo laserinfo;  // 라이다 관련 상수를 저장하는 객체 변수
    float* laserData;           // 라이다 데이터를 저장하는 배열 포인터

    Twist twistMsg;     // 보낼 메시지를 저장하는 변수
    LaserScan laserMsg; // 받은 메시지를 저장하는 변수
    bool isLaserMsgIn;  // 레이저 메시지가 한번이라도 들어왔으면 true, 아니면 false

    int pubCycleMs; // 제어 주기를 저장하는 변수

public:
    // 생성자 선언
    carControlNode() : Node("car_control_node") {
        // 노드 이름 설정
        RCLCPP_INFO(get_logger(),"Car control Node Created");
        // twist publisher 설정
        twistPub = create_publisher<Twist>("/cmd_vel",10);
        // laser info 객체 생성
        laserinfo = core::LaserInfo();
        // flag 초기화
        isLaserMsgIn = false;
        // laser subscriber 설정
        laserSub = create_subscription<LaserScan>(
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
        pubTimer = this->create_wall_timer(
            std::chrono::milliseconds(pubCycleMs),
            std::bind(&carControlNode::pubTwistMsg, this)
        );
    }

    // 노드가 소멸될 때 멈춘다.
    ~carControlNode(){
        twistMsg.angular.z = 0;
        twistMsg.linear.x = 0;
        twistPub->publish(twistMsg);
    }

private:
    // 라이다 메시지를 기반으로 라이다 관련 상수를 설정하는 함수
    void setLaserConstInfo(){
        laserinfo.angleMax = laserMsg.angle_max;
        laserinfo.angleMin = laserMsg.angle_min;
        laserinfo.angleIncrement = laserMsg.angle_increment;
        laserinfo.dataSize = (laserinfo.angleMax - laserinfo.angleMin) / laserinfo.angleIncrement + 1;
        laserinfo.rangeMax = laserMsg.range_max;
        laserinfo.rangeMin = laserMsg.range_min;
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
            "get laser msg"
        );
    }  

    void pubTwistMsg(){
        // 레이저 정보가 들어오기 전까지 아무것도 하지 않는다.
        if(isLaserMsgIn){
            int dataSize = laserMsg.ranges.size();
            laserData = new float[dataSize];
            
            // 이곳에 core code 함수를 작성한다. 이 함수는 다음 입력과 출력을 가진다.
            // 입력 : laserData, laserInfo
            // 출력 : ackermanOut
            twistMsg.angular.z = 0;
            twistMsg.linear.x = 0.7;

            // publish한다.
            twistPub->publish(twistMsg);
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