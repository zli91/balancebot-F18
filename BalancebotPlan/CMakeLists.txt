cmake_minimum_required(VERSION 3.8)

project(A*algorithms)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -pedantic --std=c++11")

set(Astar_test
        astar.h
        astar.cpp
        main.cpp
        )
add_executable(Astar.exe ${Astar_test})

#set(AWAstar_test
#        AWAstar.h
#        AWAstar.cpp
#        main_awastar.cpp
#        )
#add_executable(AWAstar.exe ${AWAstar_test})
#
#set(ARAstar_test
#        ARAstar.h
#        ARAstar.cpp
#        main_arastar.cpp
#        )
#add_executable(ARAstar.exe ${ARAstar_test})