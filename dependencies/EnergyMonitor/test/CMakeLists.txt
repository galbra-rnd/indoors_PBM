add_executable(EnergyMonitorSimple_test
EnergyMonitorSimpleTest.cpp
)

add_test(NAME EnergyMonitorSimple_test COMMAND EnergyMonitorSimple_test)

# message(FATAL_ERROR ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(EnergyMonitorSimple_test PUBLIC 
energy_monitor 
gtest
)

################################################################################
set(NEW_TEST EnergyMonitorMediator_test)
add_executable(EnergyMonitorMediator_test
EnergyMonitorMediatorTest.cpp
)

add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})

target_link_libraries(${NEW_TEST} PUBLIC 
energy_monitor
gtest
)

################################################################################
set(NEW_TEST EnergyMonitorMediatorDataManager_test)
add_executable(EnergyMonitorMediatorDataManager_test
EnergyMonitorMediatorDataProviderTest.cpp
)

add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})

target_link_libraries(${NEW_TEST} PUBLIC 
energy_monitor
gtest
)

################################################################################
set(NEW_TEST EnergyMonitorTree_test)
add_executable(EnergyMonitorTree_test
EnergyMonitorTreeTest.cpp
)

add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})
# target_include_directories(${NEW_TEST} PUBLIC mocks)
target_link_libraries(${NEW_TEST} PUBLIC 
energy_monitor
gtest
# gmock
)
