# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

#项目信息、
project(MPC_trajectory)

# CMake module(s) path
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})


# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

# 添加子目录
add_subdirectory(map)
add_subdirectory(solver)
add_subdirectory(vehicle)
add_subdirectory(data)
add_subdirectory(test)

include_directories(
  ${PROJECT_SOURCE_DIR}/map
  ${PROJECT_SOURCE_DIR}/solver
  ${PROJECT_SOURCE_DIR}/vehicle
  ${PROJECT_SOURCE_DIR}/data
  ${PROJECT_SOURCE_DIR}/test
  ${catkin_INCLUDE_DIRS}
)

# 指定生成目标 
add_executable(main main.cc)

# 添加链接库
target_link_libraries(main map solver vehicle data test ${catkin_LIBRARIES} casadi)

