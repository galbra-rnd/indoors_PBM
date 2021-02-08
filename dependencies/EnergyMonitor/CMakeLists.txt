cmake_minimum_required(VERSION 3.0.2)
project(energy_monitor VERSION "0.0.03")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

configure_file(config/cmake/EnergyMonitorConfig.hpp.in ${CMAKE_BINARY_DIR}/EnergyMonitorConfig.hpp @ONLY)

## Adding BehaviorTrees as subdir.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(lib/BehaviorTree)


## Setting sources for EnergyMonitor
set(SOURCES
${CMAKE_SOURCE_DIR}/src/EnergyMonitor.cpp
${CMAKE_SOURCE_DIR}/src/EnergyMonitorMediator/EnergyMonitorMediator.cpp

${CMAKE_SOURCE_DIR}/src/TreeNodes/IsBatteryOkConditionNode.cpp
${CMAKE_SOURCE_DIR}/src/TreeNodes/IsGoHomeOkConditionNode.cpp
)

add_library(${PROJECT_NAME} 
    ${SOURCES}
)

## Linking Behavior trees
add_dependencies(${PROJECT_NAME} behaviortree_cpp_v3)
target_link_libraries(${PROJECT_NAME} behaviortree_cpp_v3)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER include/EnergyMonitor.hpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC src)

add_subdirectory(test)
add_subdirectory(lib/googletest)