# car-control 패키지

이 패키지는 f1tenth 시뮬레이션 환경에서 차를 제어하기 위한 노드(car_control)를 담고있습니다.

이 노드는 다음 토픽을 subscribe 합니다.  
>토픽 이름 : `/scan`  
토픽 유형 : [LaserScan](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)  

또한, 다음 토픽을 publish합니다.  
>토픽 이름 : `/drive`  
토픽 유형: [AckermannDriveStamped](https://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html)  

## 사용 방법
1. [도커 파일](/Dockerfile)을 이용하여 작업 환경을 구성하거나, 직접 작업 환경을 구성할 수 있습니다. 만약 도커 파일을 이용하여 작업 환경을 구성한다면, 다음과 같이 해주세요. 작업환경은 이미 구성되어있다면, 4번부터 진행해주세요.
    
2. 우선, 도커 파일이 있는 디렉토리에서 다음 명령어를 입력합니다. (도커파일을 빌드하여 이미지를 생성합니다.)
    ```bash
    docker build -t <원하는 이미지 이름> .
    ```
3. 만든 이미지 파일로 컨테이너를 생성 및 실행합니다. 
    ```bash
    xhost +local:
    docker run -it --rm \
    --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v <마운트할 디렉토리 주소>:/ws -w /ws \
    -e DISPLAY=$DISPLAY <이미지 이름>
    ```
    이때 gpu가 없는 환경이라면 `--gpus all`을 빼주세요.  
    컨테이너 접속하면 ws라는 디렉토리에 들어가게 됩니다. 이곳에 작업환경을 구성하면 마운트한 디렉토리에도 반영됩니다.

2. [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)에서 시키는 대로 작업 환경을 구성합니다.

4. 패키지를 git clone합니다.
    ```bash
    cd ~/sim_ws/src
    git clone https://github.com/JinooLi/car_control.git
    ```

5. 작업 환경을 세팅합니다.
    ```bash
    . /opt/ros/foxy/setup.bash
    cd ~/sim_ws
    . install/setup.bash
    ```

6. 패키지를 build합니다.
    ```bash
    colcon build --packages-select car_control
    ```

7. 터미널을 두 개 켜, 두 터미널에 모두 작업환경을 세팅합니다.
    ```bash
    . /opt/ros/foxy/setup.bash
    cd ~/sim_ws
    . install/setup.bash
    ```

8. 각각의 터미널에 다음 명령어들을 입력합니다.
    - 터미널 1 : 시뮬레이션 실행
        ```bash
        ros2 launch f1tenth_gym_ros gym_bridge_launch.py
        ```
    - 터미널 2 : car_control 노드 실행
        ```bash
        ros2 run car_control car_control
        ```

- 시뮬레이션 속 차량은 간단한 PD 제어로 트랙을 따라갑니다.

### 로직 변경
- [core.cpp](/src/core.cpp)에 있는 `controlOnce()`함수 내부를 편집하면 됩니다. 
- 차량 제어 주기를 바꾸고 싶으면, [core.cpp](/src/core.cpp)의 `ControlCar()` 생성자 함수 내부에서 설정해주세요.
- 되도록 [core.cpp](/src/core.cpp)만 편집해주세요.
