cmake_minimum_required(VERSION 3.7)
project(Pendulum)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -lncurses -pthread -std=c++0x -O3")
set(SOURCE_FILES
        src/impl/list_ports/list_ports_linux.cc
        src/impl/unix.cc
        src/serial.cc
        src/SerialCommunicator.cpp
        src/LowPassFilter.cpp
        src/Controller.cpp
        main.cpp
        )
add_executable(Pendulum ${SOURCE_FILES})