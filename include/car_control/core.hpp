#ifndef CORE_H
#define CORE_H

#include <vector>

/// @brief 차량 제어의 핵심 데이터와 로직을 가지는 name space
namespace core {
/// @brief 라이다 데이터를 해석하는 데 필요한 모든 라이다 정보를 담는 class
class LaserInfo {
   public:
    int dataSize;          // 라이다 데이터 배열의 크기
    float angleMin;        // 라이다 최소 각도 [rad]
    float angleMax;        // 라이다 최대 각도 [rad]
    float angleIncrement;  // 라이다 측정 각간 각도 [rad]
    float rangeMin;        // 라이다 최소 측정 거리 [m]
    float rangeMax;        // 라이다 최대 측정 거리 [m]
};

/// @brief 속도와 조향각을 가지는 클래스.
class AckermanOut {
   public:
    float velocity;  // 속도[m/s] (앞 방향이 양수.)
    float accel;     // 가속도[m/s^2] (앞 방향이 양수.)
    float jerk;      // 가속도 변화율[m/s^3] (앞 방향이 양수.)
    float steering_angle;     // 조향각[rad] (왼쪽이 +, 오른쪽이 -)
    float steering_angle_velocity;  // 조향각 변화 속도[rad/s]
    AckermanOut() {
        velocity = 0;
        accel = 0;
        jerk = 0;
        steering_angle = 0;
        steering_angle_velocity = 0;
    }
};

/// @brief 차를 제어하기 위해 필요한 함수와 상수를 저장하는 class
class ControlCar {
   private:
    int controlCycleMs;     // 조향각과 속도를 제어하는 주기(단위:ms)
    LaserInfo laserInfo;    // 라이다 정보를 저장하는 class 변수
    bool isLaserInfoReady;  // 라이다 정보를 받아왔는지 확인하기 위한 flag
   public:
    /// @brief ControlCar 클래스의 생성자. 필요한 상수를 설정한다.
    ControlCar();

    /// @brief 라이다 정보를 저장한다.
    /// @param laserInfo 라이다 데이터를 해석하는 데 필요한 모든 라이다 정보.
    void setLaserInfo(LaserInfo _laserInfo);

    /// @brief 라이다 각도를 입력하면 그에 해당하는 라이다 데이터 인덱스가 나오는 함수
    /// @param angle 라이다 각도[rad](중앙이 0 왼쪽으로 가면 + 오른쪽으로 가면 -)
    /// @return `int index` - 해당 데이터의 인덱스
    int angleToIndexInLaser(float angle);

    /// @brief 생성자에서 설정한 제어 주기를 가져오는 함수
    /// @return 제어주기(ms)를 반환한다.
    int getControlCycleMs() const;

    /// @brief 한 번의 제어 입력을 만들어 내는 로직. 한 제어 주기에 한 번 실행되는 함수이다.
    /// @param laserData 라이다 데이터. 배열로 되어있다.
    /// @return `AckermanOut` : 속도와 조향각으로 구성된 class 객체.
    AckermanOut controlOnce(std::vector<float> laserData);
};
}  // namespace core

#endif