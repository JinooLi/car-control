#ifndef CORE_H

#define CORE_H

class LaserInfo {   // 라이다 관련 상수를 저장하는 class
public:
    int lastIndex;          // 라이다 데이터 배열의 마지막 인덱스
    int firstIndex;         // 라이다 데이터 배열의 첫번째 인덱스
    float angleMin;         // 라이다 최소 각도 [rad]
    float angleMax;         // 라이다 최대 각도 [rad]
    float angleIncrement;   // 라이다 측정 각간 각도
};



#endif