#!/bin/bash
# 设置动态连接路径
export LD_LIBRARY_PATH=$(pwd)/lib:$(pwd)/lib-usr-local:$(pwd)/lib-x86_64-linux-gnu:$LD_LIBRARY_PATH 
# 运行可执行文件
# 检查是否传递了参数
if [ -z "$1" ]; then
    sim_mode="webots"
else
    sim_mode=$1
    echo "simulation selected: $sim_mode"
fi


gdb ./mck 
# ./mck configs/config.lua


