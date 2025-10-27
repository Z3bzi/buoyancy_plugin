cmake_minimum_required(VERSION 3.5)
project(buoyancy_plugin)

find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/buoyancy_plugin.cc)
target_include_directories(${PROJECT_NAME} PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ignition-math6::ignition-math6)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
