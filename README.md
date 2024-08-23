# car-control 패키지

이 패키지는 f1tenth 시뮬레이션 환경에서 차를 제어하기 위한 노드(car_control)를 담고있습니다.

이 노드는 다음 토픽을 subscribe 합니다.  
[LaserScan](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)  

또한, 다음 토픽을 publish합니다.  
[Twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)  

## 사용 방법
- 우선 [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)에서 시키는 대로 작업 환경을 구성합니다.

- 패키지를 git clone합니다.
    ```bash
    cd ~/sim_ws/src
    git clone https://github.com/JinooLi/car_control.git
    ```

- 작업 환경을 세팅합니다.
    ```bash
    . /opt/ros/foxy/setup.bash
    cd ~/sim_ws
    . install/setup.bash
    ```

- 패키지를 build합니다.
    ```bash
    colcon build --packages-select car_control
    ```

- 터미널을 두 개 켜, 두 터미널에 모두 작업환경을 세팅합니다.
    ```bash
    . /opt/ros/foxy/setup.bash
    cd ~/sim_ws
    . install/setup.bash
    ```

- 각각의 터미널에 다음 명령어들을 입력합니다.
    - 터미널 1 : 시뮬레이션 실행
        ```bash
        ros2 launch f1tenth_gym_ros gym_bridge_launch.py
        ```
    - 터미널 2 : car_control 노드 실행
        ```bash
        ros2 run car_control car_control
        ```

- 시뮬레이션 속 차량은 직진하다가 전방에 있는 벽(또는 장애물)과의 간격이 일정 수준 이하로 떨어지면 멈추게 됩니다.

### 로직 변경
- [core.cpp](/src/core.cpp)에 있는 `controlOnce()`함수 내부를 편집하면 됩니다. 
- 차량 제어 주기를 바꾸고 싶으면, [core.cpp](/src/core.cpp)의 `ControlCar()` 생성자 함수 내부에서 설정해주세요.
- 되도록 [core.cpp](/src/core.cpp)만 편집해주세요.
