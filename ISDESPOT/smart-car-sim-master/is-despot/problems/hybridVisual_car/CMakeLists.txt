cmake_minimum_required(VERSION 3.9)
project(smart_car)

set(CMAKE_CXX_STANDARD 11)

add_executable(smart_car
        src/collision.cpp
        src/connector.h
        src/coord.h
        src/disabled_util.h
        src/math_utils.cpp
        src/math_utils.h
        src/param.cpp
        src/param.h
        src/Path.cpp
        src/Path.h
        src/ped_pomdp.cpp
        src/ped_pomdp.h
        src/simulator.cpp
        src/state.h
        src/WorldModel.cpp
        src/WorldModel.h)
