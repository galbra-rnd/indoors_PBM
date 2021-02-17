cmake_minimum_required(VERSION 3.0.2)
project(control_provider VERSION "0.0.02")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

configure_file(config/cmake/ControlProviderConfig.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/ControlProviderConfig.hpp @ONLY)


set(SOURCES
${CMAKE_CURRENT_SOURCE_DIR}/src/ControlProviderMediator/ControlProviderMediator.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/TreeNodes/LandActionNode.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/TreeNodes/GoHomeActionNode.cpp
)

add_library(${PROJECT_NAME} 
    ${SOURCES}
)



set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)





if("${CONTROL_PROVIDER_BUILD_TYPE}" STREQUAL "Debug")
    message(FATAL_ERROR "Using debug")

    find_package(behaviortree_cpp_v3 PATHS ${CMAKE_SOURCE_DIR}/../../../../devel/lib/libbehaviortree_cpp_v3)
    target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/../lib/BehaviorTree/include)
    target_link_libraries(${PROJECT_NAME} spdlog)
    add_subdirectory(lib/googletest)
    # add_subdirectory(lib/spdlog)
    add_subdirectory(test)
else()
    ## Linking Behavior trees
    add_dependencies(${PROJECT_NAME} behaviortree_cpp_v3)
    target_link_libraries(${PROJECT_NAME} behaviortree_cpp_v3)
    
    add_dependencies(${PROJECT_NAME} spdlog)
    # add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/spdlog)
    target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/../devel/lib/libspdlogd.a)
    target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../lib/spdlog/include)
    
endif("${CONTROL_PROVIDER_BUILD_TYPE}" STREQUAL "Debug")


# find_package(GTest)
# target_link_libraries(${PROJECT_NAME} GTest)
# add_dependencies(${PROJECT_NAME} ../lib/googletest/gtest)
# find_package(behaviortree_cpp_v3 PATHS /home/msi/catkin_ws/devel/lib/libbehaviortree_cpp_v3)
# add_subdirectory(lib/googletest)