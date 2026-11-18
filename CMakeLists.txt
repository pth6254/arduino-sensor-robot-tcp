cmake_minimum_required(VERSION 3.10)
project(TcpIpLab CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 실행 파일을 build/ 바로 아래에 모음
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# 실습 과제 1: 센서 데이터 모니터링 (port 8080)
add_executable(sensor_server sensor/sensor_server.cpp)
add_executable(sensor_client sensor/sensor_client.cpp)

# 실습 과제 2: 4축 로봇암 원격 제어 (port 8081)
add_executable(robot_server robot/robot_server.cpp)
add_executable(robot_client robot/robot_client.cpp)

# 통합 서버: 보드 하나로 센서 + 로봇암 동시 (port 8080 + 8081)
add_executable(integrated_server integrated/integrated_server.cpp)
