#include "../include/car_control/core.hpp"

#include <cmath>
#include <memory>

const float pi = 3.14159265359;
const float ratioConst = 0.01;
const float middleRatio = 0.01;

using namespace core;

// ================================여기에서 로직 작업!============================

ControlCar::ControlCar() {
    this->isLaserInfoReady = false;
    this->controlCycleMs = 10;  // 제어 주기 꼭 설정!!!!!!!!!!!!!!!!!!!!!!!!!
}


AckermanOut ControlCar::controlOnce(std::vector<float> laserData) {
    AckermanOut out = AckermanOut();

    int frontIndex = angleToIndexInLaser(0);
    int lIndex = angleToIndexInLaser(pi / 2);
    int rIndex = angleToIndexInLaser(-pi / 2);
    int lfIndex = angleToIndexInLaser(pi / 4);
    int rfIndex = angleToIndexInLaser(-pi / 4);

    if (laserData[frontIndex] < 1) {
        out.steering_angle = 0;
        out.velocity = 0;
    } else {
        float leftRatio = laserData[lfIndex] / laserData[lIndex];
        float rightRatio = laserData[rfIndex] / laserData[rIndex];

        float error = (leftRatio - rightRatio) * ratioConst +
                      (laserData[lIndex] - laserData[rIndex]) * middleRatio;
        out.steering_angle = error * 5;

        out.velocity = 2 / exp(abs(out.steering_angle));
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
