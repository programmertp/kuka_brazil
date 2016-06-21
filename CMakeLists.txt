cmake_minimum_required(VERSION 2.8.3)
project(kuka_brazil)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs kuka_brazil_msgs roscpp nav_msgs tf actionlib move_base_msgs)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package()

include_directories(
    include  
    ${catkin_INCLUDE_DIRS}
)

add_executable(table_detector src/table_detector.cpp)
target_link_libraries(table_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(move_to_table src/move_to_table.cpp src/RobotClass.cpp)
target_link_libraries(move_to_table ${catkin_LIBRARIES})

add_executable(automapping src/automapping.cpp src/auto_map_v2.cpp src/RobotClass.cpp)
target_link_libraries(automapping ${catkin_LIBRARIES})

add_executable(exploration_server src/exploration.cpp src/expl_helper.cpp)
target_link_libraries(exploration_server ${catkin_LIBRARIES})
