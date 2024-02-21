cmake_minimum_required(VERSION 3.0.2)
project(stimulus_signal_publisher)

find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        roscpp
        std_msgs
        )

add_service_files(
   FILES
   SetPublisherStatus.srv
 )

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        roscpp
        std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(stimulus_signal_publisher src/main.cpp)

target_link_libraries(stimulus_signal_publisher
   ${catkin_LIBRARIES}
 )
