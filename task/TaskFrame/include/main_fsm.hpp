/**
 * @file mainfsm.hpp
 * @brief 
 * @author Yonglin Chi (chiyonglin@bbeamtech.com)
 * @version 0.1
 * @date 2025-01-09
 * 
 * ©2024 Balance Beam Tech Ltd.  All rights reserved.
 * 
 * @par        changelist:
 * Date        Version  Author         Description
 * 2025-01-09  0.1      Yonglin Chi       first edition
 */

#ifndef __MAIN_FSM_HPP__
#define __MAIN_FSM_HPP__



#include "tinyfsm.hpp"
#include "utilbase/logger.h"

// 定义事件基类
struct MainEvent : tinyfsm::Event
{
};



// 模式切换事件，用于手动与自动模式的切换
struct Manual_Mode : MainEvent{
  Manual_Mode() { messages = "manual_mode"; };
};
struct Auto_Mode : MainEvent{
  Auto_Mode() { messages = "auto_mode"; };
};



// 各种具体事件，继承自 MainEvent

struct FSM_Init_Event : MainEvent
{
  FSM_Init_Event() { messages = "FSM_Init_Event"; };
};

struct FSM_Config_Event : MainEvent
{
  FSM_Config_Event() { messages = "FSM_Config_Event"; };
};

struct FSM_MotorOn_Event : MainEvent
{
  FSM_MotorOn_Event() { messages = "FSM_MotorOn_Event"; };
};

struct FSM_ControlOn_Event : MainEvent
{
  FSM_ControlOn_Event() { messages = "FSM_ControlOn_Event"; };
};

struct FSM_Error_Event : MainEvent
{
  FSM_Error_Event() { messages = "ERROR_STATE"; };
};

struct FSM_ControlOff_Event : MainEvent
{
  FSM_ControlOff_Event() { messages = "FSM_ControlOff_Event"; };
};

struct FSM_MotorOff_Event : MainEvent
{
  FSM_MotorOff_Event() { messages = "Motor_off"; };
};



struct FSM_Emergency_Stop_Event : MainEvent
{
  FSM_Emergency_Stop_Event() { messages = "Emergency_Stop_Event "; };
};

struct FSM_Reset_Event : MainEvent
{
  FSM_Reset_Event() { messages = "FSM_Reset_Event"; };
};





// 主状态机
template <typename Task>
class ESTOP_STATE;

template <typename Task>
class MOTION_STOPPING_STATE;

template <typename Task>
class BaseFSM : public tinyfsm::Fsm<BaseFSM<Task>>
{

private:
   static Task * _active_task;

public:

  static void react(MainEvent const &){Log_D("BaseFSM: Undefined event\n"); }

  // 主状态机事件的具体处理
  virtual void react(FSM_Init_Event const & e) { Log_D("BaseFSM: FSM_Init_Event. \n");}
  virtual void react(FSM_Config_Event const & e) { Log_D("BaseFSM: FSM_Config_Event. \n");}
  virtual void react(FSM_MotorOn_Event const & e) { Log_D("BaseFSM: FSM_MotorOn_Event. \n");}
  virtual void react(FSM_MotorOff_Event const &) { Log_D("BaseFSM: FSM_MotorOff_Event.\n");}
  virtual void react(FSM_ControlOn_Event const &) { Log_D("BaseFSM: FSM_ControlOn_Event.\n");}
  virtual void react(FSM_ControlOff_Event const &) { Log_D("BaseFSM: FSM_ControlOff_Event.\n");}
  virtual void react(FSM_Error_Event const &) { Log_D("BaseFSM: FSM_Error_Event.\n");}
  virtual void react(FSM_Reset_Event const &) { Log_D("BaseFSM: FSM_Reset_Event.\n");}

  virtual void react(FSM_Emergency_Stop_Event const &)
  {
    Log_D("BaseFSM: FSM_Emergency_Stop_Event. \n");
    if (getTask() != nullptr)
      getTask()->EstopAction();
    else
      Log_E("BaseFSM: FSM_Emergency_Stop_Event but task is null. \n");
    // 急停是安全锁存：EstopAction 失败也不能阻止进入 ESTOP
    this->template transit<ESTOP_STATE<Task>>();
  }

  // 进入和退出主状态机的操作
  virtual void entry(void) { Log_D("Entering BaseFSM. \n"); }
  virtual void exit(void) { Log_D("Exiting BaseFSM\n"); }

  virtual void run(void) { ;} //Log_D("Execute the BaseFSM main loop. \n"); }


  static void setTask(Task * task)
  {
    _active_task = task;
  };

  static Task * getTask()
  {
    if (_active_task == nullptr)
    {
       Log_W("MainFSM: Task is not set. \n");
       return nullptr;    
    }
    
    return _active_task;
  };


};




// 状态机的实现
template <typename Task>
class MOTION_STATE;

template <typename Task>
class UNCONFIGED_STATE;

template <typename Task>
class MOTOROFF_STATE;

template <typename Task>
class ERROR_STATE;

template <typename Task>
class READY_STATE;

template <typename Task>
class UNINITIATED_STATE;

template <typename Task>
class AutoMode;

template <typename Task>
class PReady;

template <typename Task>
class Running;

template <typename Task>
class Reset;


template <typename Task>
Task * BaseFSM<Task>::_active_task = nullptr;

//启动未初始化状态
template <typename Task>
class UNINITIATED_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;

    void entry() override {Log_D("Entering UNINITIATED_STATE. \n");}

    virtual void exit(void) { Log_D("Exiting UNINITIATED_STATE. \n");}

    void react(MainEvent const &){Log_D("UNINITIATED_STATE: Undefined event\n"); }

    void react(FSM_Init_Event const &e)
    {
      Log_D("UNINITIATED_STATE: FSM_Init_Event. \n" );
      if(base::getTask() != nullptr)
      {
        if(base::getTask()->InitAction() == 0)
          base::template transit<UNCONFIGED_STATE<Task>>();
        else
          Log_D("UNINITIATED_STATE: event handling error of FSM_Init_Event. \n" );
      }

    }

    void react(FSM_Error_Event const &e)
    {
      Log_D("UNINITIATED_STATE: FSM_Error_Event event. \n" );
      if(base::getTask() != nullptr)
        base::getTask()->ErrorAction();
      base::template transit<ERROR_STATE<Task>>();
    }


};

//启动未初始化状态
template <typename Task>
class UNCONFIGED_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;

    void entry() override {Log_D("Entering UNCONFIGED_STATE. \n");}

    virtual void exit(void) { Log_D("Exiting UNCONFIGED_STATE. \n");}

    void react(MainEvent const &){Log_D("UNCONFIGED_STATE: Undefined event\n"); }

    void react(FSM_Config_Event const &e)
    {
      Log_D("UNCONFIGED_STATE: FSM_Config_Event. \n" );
      if(base::getTask() != nullptr)
      {
        if(base::getTask()->ConfigAction()==0)
          base::template transit<MOTOROFF_STATE<Task>>();
        else
          Log_D("UNCONFIGED_STATE: event handling error of FSM_Config_Event. \n" );
      }

    }

    void react(FSM_Error_Event const &e)
    {
      Log_D("UNCONFIGED_STATE: FSM_Error_Event event. \n" );
      if(base::getTask() != nullptr)
        base::getTask()->ErrorAction();
      base::template transit<ERROR_STATE<Task>>();
    }


};


template <typename Task>
class MOTOROFF_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;
    void entry() override
    {
        Log_D("Entering MOTOROFF_STATE. \n");

    }

    void exit() override
    {
        Log_D("Exiting MOTOROFF_STATE. \n");
    }

    void react(FSM_Error_Event const &e)
    {
        Log_D("MOTOROFF_STATE: FSM_Error_Event event. \n" );
        if(base::getTask() != nullptr)
          base::getTask()->ErrorAction();
        base::template transit<ERROR_STATE<Task>>();
    }
      // 主状态机事件的具体处理
    void react(FSM_MotorOn_Event const & e) 
    { 
    Log_D("MainFSM: FSM_MotorOn_Event. \n");
    if(base::getTask() != nullptr)
    {
        if(base::getTask()->MotorOnAction() ==0)
            base::template transit<READY_STATE<Task>>();
        else
            Log_D("MOTOROFF_STATE: event handling error of FSM_MotorOn_Event. \n" );
    }
        
  }

};


template <typename Task>
class READY_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;
    void entry() override
    {
        Log_D("Entering READY_STATE. \n");
    }

    void exit() override
    {
        Log_D("Exiting READY_STATE. \n");
    }

    void react(FSM_ControlOn_Event const &e)
    {
        Log_D("READY_STATE: FSM_ControlOn_Event event. \n" );
        if(base::getTask() != nullptr)
        {
            if(base::getTask()->ControlOnAction() == 0)
                base::template transit<MOTION_STATE<Task>>();
            else
              Log_D("READY_STATE: event handling error of FSM_ControlOn_Event. \n" );
        }            
    }

    void react(FSM_Error_Event const &e)
    {
        Log_D("READY_STATE: FSM_Error_Event event. \n" );
        if(base::getTask() != nullptr)
          base::getTask()->ErrorAction();
        base::template transit<ERROR_STATE<Task>>();

    }
    void react(FSM_MotorOff_Event const &e)
    {
        Log_D("READY_STATE: FSM_MotorOff_Event event. \n" );
        if(base::getTask() != nullptr)
          if(base::getTask()->MotorOffAction() != 0)
            Log_D("READY_STATE: event handling error of FSM_MotorOff_Event. \n" );
        base::template transit<MOTOROFF_STATE<Task>>();

    }

    void run() override
    {
        // Log_D("READY_STATE: run. \n" );
        if(base::getTask() != nullptr)
          base::getTask()->ReadyStateRun();
    }
};


template <typename Task>
class MOTION_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;
    void entry() override
    {
        Log_D("Entering MOTION_STATE. \n");
    }

    void exit() override
    {
        Log_D("Exiting MOTION_STATE. \n");
    }

    void react(FSM_ControlOff_Event const &e)
    {
        Log_D("MOTION_STATE: FSM_ControlOff_Event event. \n" );
        if(base::getTask() != nullptr)
        {
            if(base::getTask()->ControlOffAction() != 0)
              Log_D("MOTION_STATE: event handling error of FSM_ControlOff_Event. \n" );
        }
        base::template transit<READY_STATE<Task>>();

    }
    void react(FSM_Error_Event const &e)
    {
        Log_D("MOTION_STATE: FSM_Error_Event event. \n" );
        if(base::getTask() != nullptr && base::getTask()->StartEmergencyPlannedStop())
        {
            MOTION_STOPPING_STATE<Task>::SetTargetEstop(false);
            base::template transit<MOTION_STOPPING_STATE<Task>>();
            return;
        }
        if(base::getTask() != nullptr)
          base::getTask()->ErrorAction();
        base::template transit<ERROR_STATE<Task>>();

    }

    // 重写基类默认：MOTION 中急停先规划停，停完才 EstopAction + 进 ESTOP
    void react(FSM_Emergency_Stop_Event const &e) override
    {
        Log_D("MOTION_STATE: FSM_Emergency_Stop_Event event. \n");
        if(base::getTask() != nullptr && base::getTask()->StartEmergencyPlannedStop())
        {
            MOTION_STOPPING_STATE<Task>::SetTargetEstop(true);
            base::template transit<MOTION_STOPPING_STATE<Task>>();
            return;
        }
        if(base::getTask() != nullptr)
          base::getTask()->EstopAction();
        base::template transit<ESTOP_STATE<Task>>();
    }

    void react(FSM_MotorOff_Event const &e)
    {
        Log_D("MOTION_STATE: FSM_MotorOff_Event event. \n" );
        if(base::getTask() != nullptr)
        {
            if(base::getTask()->ControlOffAction() != 0)
              Log_D("MOTION_STATE: event handling error of FSM_MotorOff_Event. \n" );
            base::template transit<READY_STATE<Task>>();

            if(base::getTask()->MotorOffAction() != 0)
              Log_D("MOTION_STATE: event handling error of FSM_MotorOff_Event. \n" );
            base::template transit<MOTOROFF_STATE<Task>>();
        }

    }

    void run() override
    {
        // Log_D("MOTION_STATE: run. \n" );
        if(base::getTask() != nullptr)
          base::getTask()->MotionStateRun();
    }
};


// 中间停机态：MOTION 收到 ERROR/ESTOP 且需要规划停时进入。
// run() 每周期执行停机步进；停完才执行目标 Action 并进目标态。
// 期间忽略新的 ERROR/ESTOP 事件（停完再进）。
template <typename Task>
class MOTION_STOPPING_STATE
    : public BaseFSM<Task>
{
    using base = BaseFSM<Task>;
    static bool _to_estop;   // 停完后的目标：true=ESTOP，false=ERROR

public:
    static void SetTargetEstop(bool to_estop) { _to_estop = to_estop; }

    void entry() override { Log_D("Entering MOTION_STOPPING_STATE. \n"); }
    void exit() override { Log_D("Exiting MOTION_STOPPING_STATE. \n"); }

    void react(FSM_Error_Event const &) override
    { Log_D("MOTION_STOPPING_STATE: FSM_Error_Event ignored (stopping). \n"); }
    void react(FSM_Emergency_Stop_Event const &) override
    { Log_D("MOTION_STOPPING_STATE: FSM_Emergency_Stop_Event ignored (stopping). \n"); }

    void run() override
    {
        if(base::getTask() == nullptr) return;
        if(!base::getTask()->EmergencyPlannedStopRun()) return;
        if(_to_estop)
        {
            base::getTask()->EstopAction();
            base::template transit<ESTOP_STATE<Task>>();
        }
        else
        {
            base::getTask()->ErrorAction();
            base::template transit<ERROR_STATE<Task>>();
        }
    }
};

template <typename Task>
bool MOTION_STOPPING_STATE<Task>::_to_estop = false;



template <typename Task>
class ERROR_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;
    void entry() override
    {
        Log_D("Entering ERROR_STATE. \n");
    }

    void exit() override
    {
        Log_D("Exiting ERROR_STATE. \n");
    }

    void react(FSM_Reset_Event const &e)
    {
        Log_D("ERROR_STATE: FSM_Reset_Event event. \n" );
        if(base::getTask() != nullptr)
        {
          int result = base::getTask()->ResetAction();
            if(result == 0)
                base::template transit<MOTOROFF_STATE<Task>>();
            else if(result == 1)
                base::template transit<UNCONFIGED_STATE<Task>>();
            else
                Log_D("ERROR_STATE: event handling error of FSM_Reset_Event. \n" );
        }
    }

    void run() override
    {
        // Log_D("ERROR_STATE: run. \n" );
        if(base::getTask() != nullptr)
          base::getTask()->ErrorStateRun();
    }
};



//急停锁存状态：断主电、抱闸抱紧；仅响应人工复位
template <typename Task>
class ESTOP_STATE
    : public BaseFSM<Task>
{

    using base = BaseFSM<Task>;
    void entry() override
    {
        Log_D("Entering ESTOP_STATE. \n");
    }

    void exit() override
    {
        Log_D("Exiting ESTOP_STATE. \n");
    }

    // 已处急停：幂等，不再动作
    void react(FSM_Emergency_Stop_Event const &e)
    {
        Log_D("ESTOP_STATE: FSM_Emergency_Stop_Event ignored (already in ESTOP). \n");
    }

    void react(FSM_Reset_Event const &e)
    {
        Log_D("ESTOP_STATE: FSM_Reset_Event event. \n");
        if(base::getTask() != nullptr)
        {
            if(base::getTask()->EstopResetAction() == 0)
                base::template transit<MOTOROFF_STATE<Task>>();
            else
                Log_E("ESTOP_STATE: EstopResetAction failed, staying in ESTOP. \n");
        }
    }

    void run() override
    {
        if(base::getTask() != nullptr)
            base::getTask()->EstopStateRun();
    }
};



#endif // __MAIN_FSM_HPP__