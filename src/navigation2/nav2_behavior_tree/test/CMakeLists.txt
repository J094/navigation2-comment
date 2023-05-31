# 先测行为树转换
ament_add_gtest(test_bt_conversions test_bt_conversions.cpp)
ament_target_dependencies(test_bt_conversions ${dependencies})

# 测试 condition 节点
add_subdirectory(plugins/condition)
# 测试 decorator 节点
add_subdirectory(plugins/decorator)
# 测试 control 节点
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)
