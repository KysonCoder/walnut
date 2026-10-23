#ifndef __TRAJECTORYGROUP_HPP__
#define __TRAJECTORYGROUP_HPP__

#include "spline/spline_data.hpp"

// shm 环形队列元素：真实样条段 POD（func/spline 库）。
// 满足 RingBufferIpcData 约束：trivially copyable / standard layout /
// trivially destructible。
using PathSplineData = spline::PathSplineData;

#endif
