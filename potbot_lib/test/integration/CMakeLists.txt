find_package(ament_cmake_gtest REQUIRED)

set(TEST_LIBS
  potbot_lib_utility
)

ament_add_gtest(test_apf_pipeline test_apf_pipeline.cpp)
target_include_directories(test_apf_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
)
target_link_libraries(test_apf_pipeline ${TEST_LIBS})
