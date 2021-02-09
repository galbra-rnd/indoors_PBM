cmake_minimum_required(VERSION 3.0.2)
project(energy_monitor VERSION "0.0.03")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

configure_file(config/cmake/EnergyMonitorConfig.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/EnergyMonitorConfig.hpp @ONLY)


## Setting sources for EnergyMonitor
set(SOURCES
${CMAKE_CURRENT_SOURCE_DIR}/src/EnergyMonitor.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/EnergyMonitorMediator/EnergyMonitorMediator.cpp

${CMAKE_CURRENT_SOURCE_DIR}/src/TreeNodes/IsBatteryOkConditionNode.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/TreeNodes/IsGoHomeOkConditionNode.cpp
)

add_library(${PROJECT_NAME} 
    ${SOURCES}
)

# message(FATAL_ERROR ${CMAKE_CURRENT_SOURCE_DIR})

## Linking Behavior trees
add_dependencies(${PROJECT_NAME} behaviortree_cpp_v3)
target_link_libraries(${PROJECT_NAME} behaviortree_cpp_v3)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/EnergyMonitor.hpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_subdirectory(test)
add_dependencies(${PROJECT_NAME} gtest)
# add_subdirectory(lib/googletest)