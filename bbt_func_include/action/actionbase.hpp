/**
 * @file actionbase.hpp
 * @brief define the base data structure of SOC (single open chain) robot structure
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.2
 * @date 2026-05-09
 *
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 *
 * @par        changelist:
 * Date        Version  Author         Description
 * 2024-08-23  0.1      Yonglin Chi       first edition
 * 2026-05-09  0.2      Claude            refactor to pointer-based data, Attach/Detach pattern
 */

#ifndef __ROBOT_ACTION_BASE_H__
#define __ROBOT_ACTION_BASE_H__

#include <vector>
#include <cstring>

#include "logger.h"

#include "pose.hpp"
#include "trajectory_types.hpp"

// ============================================================================

#include "ring_buffer_121.hpp"

#include "targetgroup.hpp"

#include "actiondata.hpp"

// Forward declaration from actiondata.cpp
void InitActionData(ActionData* data);

#define ACTION_LOOKAHEAD_LENGTH 8
#define ACTION_LENGTH 8

#define JOGGING_ACTION_LENGTH 16
#define PROGRAM_ACTION_LENGTH 128

#define SOC_INSTANCE_NAME_LEN 32
#define SOC_MAX_JOINT_NUMBER 7

#define FRAME_UNITY	{{ { 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0, 0.0 } },0}


/*
 * Define default data and minimum time for a coord fine point.
 */
#define ACTION_MIN_TIME_FOR_COORD_FINEP    0.001
#define ACTION_STOPPPOINT_DEFAULT_POS_FACTOR   1.0
#define ACTION_STOPPPOINT_DEFAULT_SPEED_FACTOR 1.0
#define ACTION_STOPPPOINT_DEFAULT_MIN_TIME     0.0
#define ACTION_STOPPPOINT_DEFAULT_MAX_TIME    20.0



/**
 * @class CActionBase
 * @brief 机器人动作基类
 * @details 提供机器人运动规划的基础功能，包括：
 *          - 直线、圆弧、关节插值
 *          - 速度规划与限制计算
 *          - 目标点管理
 *          - 姿态插值与四元数运算
 *          - 基于状态机的路径规划 (PathFsm)
 *
 * @par 使用示例:
 * @code
 *   CActionBase action(ACTION_LIN);
 *   action.SetStartTarget(start);
 *   action.SetEndTarget(end);
 *   action.SetSpeed(speed_data);
 *   action.ExecutePathPlanning();
 * @endcode
 *
 * @see PathFsm 路径规划状态机
 * @see CTarget 目标点类
 */
// ============================================================================

class CActionBase
{
protected:
   ActionData* _data = nullptr;                       ///< Pointer to shared/local ActionData
   bool _owns_data = false;                           ///< True if this instance allocated _data

   // Runtime CTarget wrappers — created in Attach(), detached in ~CActionBase/Detach()
   CTarget _start_target;                             ///< 起始目标点 wrapper
   CTarget _end_target;                               ///< 终止目标点 wrapper


   /**
    * @brief 计算关节绝对距离
    * @param[in] p1 起始目标点
    * @param[in] p2 终止目标点
    * @param[out] abs_joint_dist 各关节绝对距离数组
    */
   void CalcJointsAbsDist(CTarget *p1, CTarget *p2, double abs_joint_dist[]);

   /**
    * @brief 计算关节距离
    * @param[in] p1 起始目标点
    * @param[in] p2 终止目标点
    * @param[out] linear_dist 线性距离输出
    * @param[out] rotational_dist 旋转距离输出
    */
   void CalcJointsDist(CTarget *p1, CTarget *p2, double *linear_dist, double *rotational_dist);

   /**
    * @brief 计算各关节的最大距离
    * @param[in] start_joints 起始关节角
    * @param[in] end_joints 终止关节角
    * @param[out] distance 各关节距离的最大值
    */
   void JointsMaximumDistance(SOC_JOINTS start_joints, SOC_JOINTS end_joints, double *distance);

   /**
    * @brief 计算关节长度（各关节距离平方和的平方根）
    * @param[in] start_joints 起始关节角
    * @param[in] end_joints 终止关节角
    * @param[out] length 计算得到的关节长度
    */
   void CalcJointsLength(SOC_JOINTS start_joints, SOC_JOINTS end_joints, double *length);

   /**
    * @brief 计算归一化路径增量 dL
    * @details 根据速度限制和插值周期计算单位采样时间的路径增量
    */
   void CalculateDL();

   /**
    * @brief 关节速度预处理
    * @details 计算关节运动的速度限制和归一化参数
    */
   void JointSpeedPrepare();

public:

   /**
    * @brief 默认构造函数
    * @details 创建一个空的动作对象，分配本地 ActionData
    */
   CActionBase();

   /**
    * @brief 包装现有 ActionData 的构造函数
    * @param[in] data 指向现有 ActionData 的指针
    */
   explicit CActionBase(ActionData* data);

   /**
    * @brief 拷贝构造函数
    * @details 共享同一 ActionData 指针（浅拷贝）
    */
   CActionBase(const CActionBase& other);

   /**
    * @brief 拷贝赋值运算符
    * @details 共享同一 ActionData 指针（浅拷贝）
    */
   CActionBase& operator=(const CActionBase& other);

   /**
    * @brief 析构函数
    */
   virtual ~CActionBase();

   /**
    * @brief 绑定到现有 ActionData
    * @param[in] data 指向现有 ActionData 的指针
    */
   void Attach(ActionData* data);

   /**
    * @brief 解绑 ActionData
    */
   void Detach();

   /**
    * @brief 检查是否已绑定数据
    * @return true 表示已绑定
    */
   bool IsAttached() const { return _data != nullptr; }

   /**
    * @brief 获取底层 ActionData 指针
    * @return ActionData 指针
    */
   ActionData* GetData() const { return _data; }

   /**
    * @brief 初始化动作对象
    * @details 重置所有内部状态和数据到初始值
    */
   void Init();

   /**
    * @brief 获取终止目标点
    * @return 终止目标点对象
    */
   CTarget GetEndTarget();

   /**
    * @brief 获取起始目标点
    * @return 起始目标点对象
    */
   CTarget GetStartTarget();

   /**
    * @brief 设置终止目标点
    * @param[in] end_target 终止目标点
    */
   void SetEndTarget(CTarget end_target);

   /**
    * @brief 设置起始目标点
    * @param[in] start_target 起始目标点
    * @details 同时设置起始目标点有效标志为 true
    */
   void SetStartTarget(CTarget start_target);

   /**
    * @brief 同时设置起始和终止目标点
    * @param[in] start_target 起始目标点
    * @param[in] end_target 终止目标点
    */
   void SetTargets(CTarget start_target, CTarget end_target);

   /**
    * @brief 设置动作名称
    * @param[in] soc_name 动作名称字符串（最大32字符）
    */
   void SetName(const char *soc_name);

   /**
    * @brief 关节插值预处理
    * @details 计算关节运动所需的速度、加速度等参数
    */
   void JointInterpPrepare();

   /**
    * @brief 获取插值数据
    * @return 动作插值数据结构
    */
   ACTION_INTERPOLATION_DATA GetInterpolationData();

   // virtual CActionBase & operator = (const CActionBase & other);

   /**
    * @brief 设置运动类型
    * @param[in] motion_type 运动类型枚举值
    */
   void SetMotionType(ACTION_MOTION_TYPE motion_type);

   /**
    * @brief 获取当前运动类型
    * @return 运动类型枚举值
    */
   ACTION_MOTION_TYPE GetMotionType();

   /**
    * @brief 清除动作数据
    * @details 重置所有内部状态，恢复到初始状态
    */
   void Clear();

   /**
    * @brief 设置动作速度参数
    * @param[in] speed 速度数据结构
    */
   void SetSpeed(ACTION_SPEED_DATA speed);

   /**
    * @brief 获取动作速度参数
    * @return 速度数据结构
    */
   ACTION_SPEED_DATA GetSpeed();

   /**
    * @brief 获取主轴速度（归一化值）
    * @param[out] master_speed 主轴速度输出指针
    * @details 虚函数，子类可重写以提供特定实现
    */
   virtual void GetMasterSpeed(double *master_speed);

   /**
    * @brief 获取主轴加速度（归一化值）
    * @param[out] master_acc 主轴加速度输出指针
    * @details 虚函数，子类可重写以提供特定实现
    */
   virtual void GetMasterAcc(double *  master_acc);

   /**
    * @brief 获取主轴减速度（归一化值）
    * @param[out] master_dec 主轴减速度输出指针
    * @details 虚函数，子类可重写以提供特定实现
    */
   virtual void GetMasterDec(double *  master_dec);

   /**
    * @brief 获取主轴路径长度
    * @param[out] master_path_length 路径长度输出指针
    * @details 虚函数，子类可重写以提供特定实现
    */
   virtual void GetMasterPathLength(double *master_path_length);

   /**
    * @brief 计算动作距离
    * @return 计算得到的距离值
    * @details 虚函数，子类应根据运动类型实现具体计算
    */
   virtual double CalculateDistance();

   /**
    * @brief 获取指定归一化位置处的关节角
    * @param[in] norm_pos 归一化位置参数（0.0 到 1.0）
    * @param[out] joints 输出的关节角数组
    */
   void GetJointPos(double norm_pos, SOC_JOINTS *joints);

   /**
    * @brief 计算两点间的直线距离
    * @param[in] p1 起始目标点
    * @param[in] p2 终止目标点
    * @param[out] distance 计算得到的直线距离（米）
    */
   void CalcLineDist(CTarget *p1, CTarget *p2, double *distance);

   /**
    * @brief 计算两点间的四元数姿态距离
    * @param[in] p1 起始目标点
    * @param[in] p2 终止目标点
    * @param[out] fi 旋转角度（弧度）
    * @param[out] axis 旋转轴（单位向量）
    */
   void CalcQuaternionDist(CTarget *p1, CTarget *p2, double *fi, math::Cartesian3<double> *axis);

   /**
    * @brief 初始化圆弧运动参数
    * @param[in] p_start 圆弧起始点
    * @param[in] p_circle 圆弧中间点
    * @param[in] p_end 圆弧终止点
    * @param[out] circle_radius 计算得到的圆弧半径
    * @param[out] opening_angle 计算得到的圆弧张角（弧度）
    * @param[out] start_angle 计算得到的起始角度（弧度）
    * @param[out] circle_centre 计算得到的圆心位置
    * @param[out] q_circle_rel_object 圆弧相对于工件坐标系的姿态四元数
    * @return 0 表示成功，非0表示错误码
    */
   int InitCircle(CTarget *p_start, CTarget *p_circle, CTarget *p_end,
                  double *circle_radius, double *opening_angle,
                  double *start_angle, math::Cartesian3<double> *circle_centre, math::Quaternion<double> *q_circle_rel_object);

   /**
    * @brief 初始化圆弧四元数插值参数
    * @param[in] p_start 圆弧起始点
    * @param[in] p_end 圆弧终止点
    * @param[in] opening_angle 圆弧张角（弧度）
    * @param[in] start_angle 起始角度（弧度）
    * @param[in] q_circle_rel_object 圆弧相对于工件坐标系的姿态四元数
    * @param[out] q_start_rel_circle_arc 起始点相对于圆弧的姿态
    * @param[out] q_end_rel_circle_arc 终止点相对于圆弧的姿态
    * @param[out] axis_start_end_rel_circle 起始到终止相对于圆心的轴向
    * @param[out] theta_start_end_rel_circle 起始到终止的角度
    * @return true 表示成功，false 表示失败
    */
   bool InitCircleQuat(CTarget *p_start, CTarget *p_end,
                       double opening_angle, double start_angle, math::Quaternion<double> *q_circle_rel_object, math::Quaternion<double> *q_start_rel_circle_arc,
                       math::Quaternion<double> *q_end_rel_circle_arc, math::Cartesian3<double> *axis_start_end_rel_circle, double *theta_start_end_rel_circle);

   /**
    * @brief 通用插值函数
    * @param[out] p 插值结果目标点
    * @param[in] p_start 起始目标点
    * @param[in] p_end 终止目标点
    * @param[in] L 归一化插值参数（0.0 到 1.0）
    * @param[in] method 插值方法枚举
    */
   void Interpolate(CTarget *p, CTarget *p_start, CTarget *p_end,
                    double L, enum action_ipol_method method);

   /**
    * @brief 关节空间插值
    * @param[out] p 插值结果目标点
    * @param[in] p_start 起始目标点
    * @param[in] p_end 终止目标点
    * @param[in] L 归一化插值参数（0.0 到 1.0）
    * @param[in] robot_param 机器人参数对象
    */
   void InterpolateJoints(CTarget *p, CTarget *p_start, CTarget *p_end,
                          double L);

   /**
    * @brief 四元数姿态插值（球面线性插值 SLERP）
    * @param[out] p 插值结果目标点
    * @param[in] p_end 终止目标点（包含目标姿态）
    * @param[in] L 归一化插值参数（0.0 到 1.0）
    * @param[in] fi 总旋转角度（弧度）
    * @param[in] axis 旋转轴（单位向量）
    */
   void InterpolateQuaternion(CTarget *p, CTarget *p_end,
                              double L, double fi, math::Cartesian3<double> *axis);

   /**
    * @brief 圆弧位置插值
    * @param[out] p 插值结果目标点
    * @param[in] L 归一化插值参数（0.0 到 1.0）
    * @param[in] circle_radius 圆弧半径
    * @param[in] opening_angle 圆弧张角（弧度）
    * @param[in] start_angle 起始角度（弧度）
    * @param[in] circle_centre 圆心位置
    * @param[in] q_circle_rel_object 圆弧相对于工件坐标系的姿态四元数
    * @param[in] arc_dir 圆弧方向向量
    */
   void InterpolateCircle(CTarget *p, double L, double circle_radius,
                          double opening_angle, double start_angle, math::Cartesian3<double> *circle_centre,
                          math::Quaternion<double> *q_circle_rel_object, math::Cartesian3<double> *arc_dir);

   /**
    * @brief 圆弧姿态四元数插值
    * @param[out] p 插值结果目标点
    * @param[in] L 归一化插值参数（0.0 到 1.0）
    * @param[in] opening_angle 圆弧张角（弧度）
    * @param[in] start_angle 起始角度（弧度）
    * @param[in] axis_start_end_rel_circle 起始到终止相对于圆心的轴向
    * @param[in] theta_start_end_rel_circle 起始到终止的角度
    * @param[in] q_circle_rel_object 圆弧相对于工件坐标系的姿态四元数
    * @param[in] q_start_rel_arc 起始点相对于圆弧的姿态
    */
   void InterpolateCircleQuat(CTarget *p, double L, double opening_angle,
                              double start_angle, math::Cartesian3<double> *axis_start_end_rel_circle,
                              double theta_start_end_rel_circle, math::Quaternion<double> *q_circle_rel_object, math::Quaternion<double> *q_start_rel_arc);

   /**
    * @brief 检查路径规划是否已完成
    * @return true 表示规划已完成
    */
};

#endif
