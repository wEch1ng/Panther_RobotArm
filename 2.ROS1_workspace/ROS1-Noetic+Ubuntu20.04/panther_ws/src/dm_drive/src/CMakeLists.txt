cmake_minimum_required(VERSION 3.5)

project (DM_Linux_Drive)

set(SOURCES
    damiao.h
    SerialPort.h
    test_damiao.cpp
)

add_executable(DM_Linux_Drive ${SOURCES})

