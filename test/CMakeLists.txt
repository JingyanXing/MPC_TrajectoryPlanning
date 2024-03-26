# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(. DIR_LIB_SRCS)


# 生成链接库
add_library (test ${DIR_LIB_SRCS})

# 添加头文件路径，包含vehicle文件夹的头文件路径
target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../vehicle 
                                        ${CMAKE_CURRENT_SOURCE_DIR}/../data)

# 如果test目标依赖于vehicle目标生成的库，需要链接vehicle库
target_link_libraries(test vehicle data)