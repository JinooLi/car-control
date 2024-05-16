#include <memory>
#include <vector>

#include "../include/car_control/core.hpp"

using namespace core;

// ================================여기에서 로직 작업!============================

ControlCar::ControlCar(){
    this->isLaserInfoRedy = false;
    this->controlCycleMs = 10; // 제어 주기 꼭 설정!!!!!!!!!!!!!!!!!!!!!!!!!
}

AckermanOut ControlCar::controlOnce(std::vector<float> laserData){
    AckermanOut out = AckermanOut();

    int index = angleToIndexInLaser(0);

    if(laserData[index] < 1){
        out.steer = 0;
        out.velocity = 0;
    }else {
        out.steer = 0;
        out.velocity = 0.9;
    }


    return out;
}


// ===========================================================================

int ControlCar::angleToIndexInLaser(float angle){
    if(angle < this->laserInfo.angleMin)return 0;
    else if(angle > this->laserInfo.angleMax)return laserInfo.dataSize - 1;
    else{
        return (angle - this->laserInfo.angleMin)/this->laserInfo.angleIncrement;
    }
}

void ControlCar::setLaserInfo(LaserInfo _laserInfo){
    this->laserInfo = _laserInfo;
    this->isLaserInfoRedy = true;
}

int ControlCar::getControlCycleMs() const{
    return this->controlCycleMs;
}

