#include <memory>
#include <vector>

#include "../include/car_control/core.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Ackermann = ackermann_msgs::msg::AckermannDriveStamped;
using LaserScan = sensor_msgs::msg::LaserScan;

class carControlNode : public rclcpp::Node {
   private:
    rclcpp::Publisher<Ackermann>::SharedPtr ackermannPub;  // ackermann publisher
    rclcpp::Subscription<LaserScan>::SharedPtr laserSub;   // laser subscriber
    rclcpp::TimerBase::SharedPtr pubTimer;                 // timer for publish cycle

    core::LaserInfo laserinfo;    // 라이다 관련 상수를 저장하는 객체 변수
    core::ControlCar controlCar;  // controlCar class 객체 변수 이 클래스가 제어의 핵심이다.

    Ackermann ackermannMsg;  // 보낼 메시지를 저장하는 변수
    LaserScan laserMsg;      // 받은 메시지를 저장하는 변수
    bool isLaserMsgIn;       // 레이저 메시지가 한번이라도 들어왔으면 true, 아니면 false

    int pubCycleMs;  // 제어 주기를 저장하는 변수

   public:
    // 생성자 선언
    carControlNode() : Node("car_control_node") {
        // controlCar 객체 생성
        controlCar = core::ControlCar();
        // laser info 객체 생성
        laserinfo = core::LaserInfo();
        // flag 초기화
        isLaserMsgIn = false;

        // 노드 이름 설정
        RCLCPP_INFO(get_logger(), "Car control Node Created");
        // ackermann publisher 설정
        ackermannPub = create_publisher<Ackermann>("/drive", 10);
        // laser subscriber 설정
        laserSub = create_subscription<LaserScan>(
            "/scan",
            10,
            std::bind(
                &carControlNode::getLaserMsg,
                this,
                std::placeholders::_1));

        // publsish 주기는 core에서 결정한다.
        pubCycleMs = controlCar.getControlCycleMs();
        pubTimer = this->create_wall_timer(
            std::chrono::milliseconds(pubCycleMs),
            std::bind(&carControlNode::pubTwistMsg, this));
    }

    // 노드가 소멸될 때 멈춘다.
    ~carControlNode() {
        ackermannMsg.drive.steering_angle = 0;
        ackermannMsg.drive.speed = 0;
        ackermannPub->publish(ackermannMsg);
    }

   private:
    // 라이다 메시지를 기반으로 라이다 관련 상수를 설정하는 함수
    void setLaserConstInfo() {
        laserinfo.angleMax = laserMsg.angle_max;
        laserinfo.angleMin = laserMsg.angle_min;
        laserinfo.angleIncrement = laserMsg.angle_increment;
        laserinfo.dataSize = (laserinfo.angleMax - laserinfo.angleMin) / laserinfo.angleIncrement + 1;
        laserinfo.rangeMax = laserMsg.range_max;
        laserinfo.rangeMin = laserMsg.range_min;
    }

    // 라이다 메시지를 받아온다.
    void getLaserMsg(const LaserScan::SharedPtr msg) {
        laserMsg = *msg;

        if (!isLaserMsgIn) {
            // 라이다 정보를 저장한다.
            setLaserConstInfo();
            // 코어에 라이다 정보를 전달한다.
            controlCar.setLaserInfo(laserinfo);
            // flag를 true로 바꿔 정보를 전달할 수 있도록 한다.
            isLaserMsgIn = true;

            RCLCPP_INFO(
                get_logger(),
                "get laser msg");
        }
    }

    void pubTwistMsg() {
        // 레이저 정보가 들어오기 전까지 아무것도 하지 않는다.
        if (isLaserMsgIn) {
            std::vector<float> laserData = laserMsg.ranges;

            // 이곳에 core code 함수를 작성한다. 이 함수는 다음 입력과 출력을 가진다.
            // 입력 : laserData / 출력 : ackermanOut
            core::AckermanOut out = this->controlCar.controlOnce(laserData);

            ackermannMsg.drive.steering_angle = out.steering_angle;
            ackermannMsg.drive.steering_angle_velocity = out.steering_angle_velocity;
            ackermannMsg.drive.speed = out.velocity;
            ackermannMsg.drive.acceleration = out.accel;
            ackermannMsg.drive.jerk = out.jerk;

            // publish한다.
            ackermannPub->publish(ackermannMsg);

            RCLCPP_INFO(
                get_logger(),
                "running!");
        } else {
            ackermannMsg.drive.steering_angle = 0;
            ackermannMsg.drive.speed = 0;
            ackermannPub->publish(ackermannMsg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<carControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}