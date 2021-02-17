
# find_package(behaviortree_cpp_v3 REQUIRED PATHS ${CMAKE_SOURCE_DIR}/../../../../devel/lib/libbehaviortree_cpp_v3)
# ################################################################################

add_executable(ContolProviderMediatorTest
ContolProviderMediatorTest.cpp
)

add_test(NAME ContolProviderMediatorTest COMMAND ContolProviderMediatorTest)

# message(FATAL_ERROR ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ContolProviderMediatorTest PUBLIC 
control_provider 
gtest
)

# ################################################################################
set(NEW_TEST LandActionNodeTest)
add_executable(LandActionNodeTest
LandActionNodeTest.cpp
)

add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})

target_include_directories(${NEW_TEST} PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_dependencies(${NEW_TEST} control_provider)
target_link_libraries(${NEW_TEST} PUBLIC 
control_provider
/home/msi/catkin_ws/devel/lib/libbehaviortree_cpp_v3.so
gtest
)

# target_include_directories(${NEW_TEST} PUBLIC ${CMAKE_SOURCE_DIR}/../lib/BehaviorTree/include)
# ################################################################################
# set(NEW_TEST EnergyMonitorMediatorDataManager_test)
# add_executable(EnergyMonitorMediatorDataManager_test
# EnergyMonitorMediatorDataProviderTest.cpp
# )

# add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})

# target_link_libraries(${NEW_TEST} PUBLIC 
# energy_monitor
# gtest
# )

# ################################################################################
# set(NEW_TEST EnergyMonitorTree_test)
# add_executable(EnergyMonitorTree_test
# EnergyMonitorTreeTest.cpp
# )

# add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})
# # target_include_directories(${NEW_TEST} PUBLIC mocks)
# target_link_libraries(${NEW_TEST} PUBLIC 
# energy_monitor
# gtest
# # gmock
# )
