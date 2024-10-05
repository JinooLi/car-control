#include "../include/car_control/core.hpp"

#include <memory>
#include <cmath>

const float pi = 3.14159265359;
const float ratioConst = 0.01;
const float middleRatio = 0.01;
const int angle_num = 30;
const float p_gain = 1;

using namespace core;

// ================================여기에서 로직 작업!============================

ControlCar::ControlCar() {
    this->isLaserInfoReady = false;
    this->controlCycleMs = 10;  // 제어 주기 꼭 설정!!!!!!!!!!!!!!!!!!!!!!!!!
}


AckermanOut ControlCar::controlOnce(std::vector<float> laserData) {
    AckermanOut out = AckermanOut();
    
    int frontIndex = angleToIndexInLaser(0);
    
    if (laserData[frontIndex] < 1) {
        out.steering_angle = 0;
        out.velocity = 0;
    } else {
        float accum_x = 0;
        float accum_y = 0;

        // 라이다 데이터를 이용하여 가야하는 방향을 계산한다.
        for (int i = 0; i < angle_num; i++) {
            float angle = -pi / 2 + pi / (angle_num - 1) * i;
            int index = angleToIndexInLaser(angle);
            float oneLaserData = laserData[index];
            if (oneLaserData > 10) {
                oneLaserData = 10;
            }

            // 구한 각도와 라이다 데이터를 각 좌표계에서 직교좌표계로 변환하여 다 더한다.
            accum_x += oneLaserData * cos(angle);  // x 좌표
            accum_y += oneLaserData * sin(angle);  // y 좌표
        }

        // 직교좌표계의 x, y 좌표를 이용하여 차량이 가야할 방향의 각도를 구한다.
        float target_angle = atan2(accum_y, accum_x);
        out.steering_angle = target_angle*p_gain;
        out.velocity = 2;
    }

    out.accel = 4;
    out.jerk = 0;

    out.steering_angle_velocity = 1;

    return out;
}

// ===========================================================================

int ControlCar::angleToIndexInLaser(float angle) {
    if (angle < this->laserInfo.angleMin)
        return 0;
    else if (angle > this->laserInfo.angleMax)
        return laserInfo.dataSize - 1;
    else {
        return (angle - this->laserInfo.angleMin) / this->laserInfo.angleIncrement;
    }
}

void ControlCar::setLaserInfo(LaserInfo _laserInfo) {
    this->laserInfo = _laserInfo;
    this->isLaserInfoReady = true;
}

int ControlCar::getControlCycleMs() const {
    return this->controlCycleMs;
}
