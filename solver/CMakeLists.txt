# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(. DIR_LIB_SRCS)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 生成链接库
add_library (solver ${DIR_LIB_SRCS})

target_link_libraries(solver ${catkin_LIBRARIES} casadi)