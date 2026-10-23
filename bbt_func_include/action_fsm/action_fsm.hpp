/**
 * @file action_fsm.hpp
 * @brief CActionFsm —— actionFSM 方法类（View，Attach/Detach）
 * @date 2026-08-27
 *
 * 持有 ActionFsmData* 指针，不负责内存生命周期；拷贝/赋值为浅拷贝。
 * 热路径（Step/PushSignal/RaiseUrgent）无堆分配、无虚函数、无异常。
 */
#ifndef __ACTION_FSM_H__
#define __ACTION_FSM_H__

#include "action_fsm_data.hpp"

class CActionFsm {
public:
    CActionFsm() = default;
    explicit CActionFsm(ActionFsmData* data) { Attach(data); }
    CActionFsm(const CActionFsm& other) = default;              // 浅拷贝 _data
    CActionFsm& operator=(const CActionFsm& other) = default;   // 浅拷贝
    ~CActionFsm() = default;                                     // 不释放 _data

    // 绑定共享数据；magic/version 不符拒绝（保持未绑定）
    void Attach(ActionFsmData* data);
    void Detach() { _data = nullptr; }
    bool IsAttached() const { return _data != nullptr; }
    ActionFsmData* GetData() const { return _data; }

    // 配置装载与校验（创建进程调用，步进前完成）。
    // 返回 FSM_OK，或 FSM_CONFIG_ERR(code, idx) 负值，或 FSM_ERR_*。
    int LoadConfig(const FsmState* states, uint8_t state_count,
                   const FsmTransition* transitions, uint8_t transition_count,
                   const FsmTrigger* triggers, uint8_t trigger_count,
                   uint8_t initial_state);

    // 信号注入（外部进程调用）。ESTOP 自动路由到紧急槽位。
    int PushSignal(FsmSignalType type, double param = 0.0);
    int RaiseUrgent(FsmSignalType type, double param = 0.0);

    // 步进（插补进程每周期调用）。未 Attach/未 configured 空转。
    void Step(const FsmContext& ctx);

    uint8_t CurrentState() const;
    bool    MotionPermitted() const;
    double  SpeedOverride() const;
    bool    FlagSet(uint8_t flag_id) const;
    void    ClearFlag(uint8_t flag_id);   // 外部进程消费后清除

private:
    ActionFsmData* _data = nullptr;
    uint32_t _inject_seq = 0;   // 注入方序号（每对象本地，调试/丢序检测用）

    int  StateIndex(uint8_t id) const;                        // -1 未找到
    int  FindTransition(uint8_t from_state, uint8_t signal) const;  // -1 无
    void ApplySignal(const FsmSignal& sig, uint64_t cycle);
    void ExecuteTransition(int trans_idx, uint64_t cycle);
    void RearmFired(uint8_t from_state, FsmSignalType sig);
    void EvaluateTriggers(const FsmContext& ctx);
    static bool EvalTrigger(const FsmTrigger& trig, const FsmContext& ctx);
};

#endif // __ACTION_FSM_H__
