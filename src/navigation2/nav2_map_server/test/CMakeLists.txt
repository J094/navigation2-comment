set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

# 单元测试
add_subdirectory(unit)
# 组件测试
add_subdirectory(component)
# cli 测试
add_subdirectory(map_saver_cli)
