#ifndef __TARGETGROUP_HPP__
#define __TARGETGROUP_HPP__

// 本头历史上是占位实现（TARGET_DATA/CTargetGroup 空壳）。真实定义在
// func/kinematic 库。这里保留转发，原因：func/action 的 actiondata.hpp
// 内部以扁平名 "targetgroup.hpp" 包含，若消费者的 include 顺序把本目录
// 排在 kinematic/ 之前会命中本文件——转发保证两种顺序下都解析到同一份
// 真实定义（include guard __TARGET_GROUP_HPP__）。
#include "kinematic/targetgroup.hpp"

#endif
