# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(. DIR_LIB_SRCS)

# 生成链接库
add_library (vehicle ${DIR_LIB_SRCS})

# 添加头文件路径，包含vehicle文件夹的头文件路径
target_include_directories(vehicle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../solver 
                                        ${CMAKE_CURRENT_SOURCE_DIR}/../map)

target_link_libraries(vehicle solver map)


