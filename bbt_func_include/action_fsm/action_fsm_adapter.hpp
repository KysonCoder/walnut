/**
 * @file action_fsm_adapter.hpp
 * @brief CActionFsmAdapter —— FSM 与插补循环之间的 header-only 便利层
 * @date 2026-08-27
 *
 * 不依赖 func/action 头文件（spec §4.4 修订）：FsmContext 由调用方
 * 插补循环按值构造，adapter 只做 Step 步进 + dL 缩放。
 */
#ifndef __ACTION_FSM_ADAPTER_H__
#define __ACTION_FSM_ADAPTER_H__

#include "action_fsm.hpp"

class CActionFsmAdapter {
public:
    void Bind(CActionFsm* fsm) { _fsm = fsm; }
    bool IsBound() const { return _fsm != nullptr; }

    // 插补循环每周期：先步进 FSM，再返回本周期生效的 dL。
    //   未绑定       → 原样返回 dL_planned（FSM 不存在时的透传行为）
    //   motion 不许可 → 0.0（冻结路径推进）
    //   否则          → dL_planned * SpeedOverride()
    double StepAndScaleDL(const FsmContext& ctx, double dL_planned) {
        if (_fsm == nullptr || !_fsm->IsAttached()) return dL_planned;
        _fsm->Step(ctx);
        if (!_fsm->MotionPermitted()) return 0.0;
        return dL_planned * _fsm->SpeedOverride();
    }

private:
    CActionFsm* _fsm = nullptr;
};

#endif // __ACTION_FSM_ADAPTER_H__
