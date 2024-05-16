#ifndef CORE_H
#define CORE_H

namespace core {
    class LaserInfo {   // 라이다 관련 상수를 저장하는 class
    public:
        int dataSize;           // 라이다 데이터 배열의 크기
        float angleMin;         // 라이다 최소 각도 [rad]
        float angleMax;         // 라이다 최대 각도 [rad]
        float angleIncrement;   // 라이다 측정 각간 각도 [rad]
        float rangeMin;         // 라이다 최소 측정 거리 [m]
        float rangeMax;         // 라이다 최대 측정 거리 [m]
    };

    class AckermanOut {
    public:
        float velocity; // 속도[m/s] (앞 방향이 양수.)
        float steer;    // 조향각[rad] (왼쪽이 음수, 오른쪽이 양수)
    };
} // namespace core


#endif