set(BINARY ${CMAKE_PROJECT_NAME}_test)

# file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

# set(SOURCES ${TEST_SOURCES})

add_executable(${BINARY}
EnergyMonitorSimpleTest.cpp
)

add_test(NAME ${BINARY} COMMAND ${BINARY})

target_include_directories(${BINARY} PRIVATE
${CMAKE_SOURCE_DIR}/include
# ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(${BINARY} PUBLIC 
${CMAKE_PROJECT_NAME} 
gtest
)