add_executable(EnergyMonitorSimple_test
EnergyMonitorSimpleTest.cpp
)

add_test(NAME EnergyMonitorSimple_test COMMAND EnergyMonitorSimple_test)

target_link_libraries(EnergyMonitorSimple_test PUBLIC 
${CMAKE_PROJECT_NAME} 
gtest
)

################################################################################
set(NEW_TEST EnergyMonitorMediator_test)
add_executable(EnergyMonitorMediator_test
EnergyMonitorMediatorTest.cpp
)

add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})

target_link_libraries(${NEW_TEST} PUBLIC 
${CMAKE_PROJECT_NAME} 
gtest
)

################################################################################
set(NEW_TEST EnergyMonitorMediatorDataManager_test)
add_executable(EnergyMonitorMediatorDataManager_test
EnergyMonitorMediatorDataManagerTest.cpp
)

add_test(NAME ${NEW_TEST} COMMAND ${NEW_TEST})

target_link_libraries(${NEW_TEST} PUBLIC 
${CMAKE_PROJECT_NAME} 
gtest
)