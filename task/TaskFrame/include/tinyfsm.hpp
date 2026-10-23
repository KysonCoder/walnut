//状态机的包括：1.进入退出函数 2.消息分发函数  3.状态转换函数 4.状态机的状态实例化模板 5.消息响应函数 6.以及目前的状态

#ifndef TINYFSM_HPP_INCLUDED
#define TINYFSM_HPP_INCLUDED

#ifndef TINYFSM_NOSTDLIB
#include <type_traits>
#include <iostream>
//std::cout << std::is_same<int, int>::value << std::endl;       // 输出：true  

#endif



namespace tinyfsm
// 命名空间主要用于组织和封装一组相关的名称（如函数、变量、类型等），以避免名称冲突。
{

  struct Event{std::string messages;};


#ifdef TINYFSM_NOSTDLIB

#else
  template <typename F, typename S>
  // 检查两个状态机是否具有相同的状态机类型
  struct is_same_fsm : std::is_same<typename F::fsmtype, typename S::fsmtype>
  {
  };
#endif
  
  template <typename S>
  struct _state_instance
  {
    using value_type = S;
    using type = _state_instance<S>;
    static S value;
  };
  // 而静态成员变量(在编译期间分配)需要一个确切的地址,下面的代码为_state_instance<S>::value提供了定义：
  template <typename S>
  typename _state_instance<S>::value_type _state_instance<S>::value;

  // --------------------------------------------------------------------------

  template <typename F>
  class Fsm
  {
  public:
    using fsmtype = Fsm<F>;
    using state_ptr_t = F *;

    static state_ptr_t current_state_ptr;

    // public, leaving ability to access state instance (e.g. on reset)
    template <typename S>
    // 声明了一个静态的、可以在编译时计算的、返回对类型 S 的对象的引用的成员函数，该函数不接受任何参数。
    static constexpr S &state(void)
    {
      // tatic_assert（关键字）在编译阶段就进行断言检查，如果条件不满足，编译就会失败，并输出提供的错误消息。
      static_assert(is_same_fsm<F, S>::value, "accessing state of different state machine");
      // 返回 _state_instance<S>::value 的引用即当前状态的实例。
      return _state_instance<S>::value;
    }

    template <typename S>
    static constexpr bool is_in_state(void)
    {
      static_assert(is_same_fsm<F, S>::value, "accessing state of different state machine");
      // 比较了 current_state_ptr 是否指向 _state_instance<S>::value 的地址。（及于当前状态是否为 S 类型）
      return current_state_ptr == &_state_instance<S>::value;
    }

    
  public:
    
    static void set_initial_state(){};

    static void reset(){};

    static void enter()
    {
      current_state_ptr->entry();
    }

    static void start()
    {
      set_initial_state();
      enter();
    }


//消息分发函数，将事件传递给当前状态，并调用当前状态的react函数
    template <typename E>
    static void dispatch(E const & event)
    {
      current_state_ptr->react(event);
    }

    
  protected:
    // 状态转移函数
    template <typename S>
    void transit(void)
    {
      static_assert(is_same_fsm<F, S>::value, "transit to different state machine");
      current_state_ptr->exit();
      current_state_ptr = &_state_instance<S>::value;
      current_state_ptr->entry();
    }
    // 离开上个状态，进入下个状态，并执行动作函数
    template <typename S, typename ActionFunction>
    void transit(ActionFunction action_function)
    {
      static_assert(is_same_fsm<F, S>::value, "transit to different state machine");
      current_state_ptr->exit();
      action_function();
      current_state_ptr = &_state_instance<S>::value;
      current_state_ptr->entry();
    }
    // 带参的transit函数，将当前状态转换到下一个状态，并执行动作函数，并检查条件函数，只有满足条件的情况下才执行转换
    template <typename S, typename ActionFunction, typename ConditionFunction>
    void transit(ActionFunction action_function, ConditionFunction condition_function)
    {
      if (condition_function())
      {
        transit<S>(action_function);
      }
    }
  };

  template <typename F>
  typename Fsm<F>::state_ptr_t Fsm<F>::current_state_ptr;

  // --------------------------------------------------------------------------


  //可变参数模板类，接受多个类型参数，管理状态机的多个状态
  template <typename... FF>
  struct FsmList;
  // 无类型参数
  //管理状态机的多个状态
  template <>
  struct FsmList<>
  {
    static void set_initial_state() {}
    static void reset() {}
    static void enter() {}
    template <typename E>
    static void dispatch(E const &) {}
  };
  // 有类型参数
  /**
   * @brief
   * 
   * @tparam F 
   * @tparam FF 
   */
  template <typename F, typename... FF>
  struct FsmList<F, FF...>
  {
    using fsmtype = Fsm<F>;

    static void set_initial_state()
    {
      fsmtype::set_initial_state();
      FsmList<FF...>::set_initial_state();
    }

    

    static void reset()
    {
      F::reset();
      FsmList<FF...>::reset();
    }

    static void enter()
    {
      fsmtype::enter();
      FsmList<FF...>::enter();
    }

    static void start()
    {
      set_initial_state();
      enter();
    }

    template <typename E>
    static void dispatch(E const &event)
    {
      fsmtype::template dispatch<E>(event);
      FsmList<FF...>::template dispatch<E>(event);
    }
  };

  //--------------------------------------------------------------------------
  //作用：重置所有状态机的状态
  //参数：无参（多个状态机的类型参数的重置）
  template <typename... SS>
  struct StateList;
  template <>
  struct StateList<>
  {
    static void reset() {}
  };
  // 有类型参数
  // 参数：S：状态机类型，SS：状态机类型列表
  template <typename S, typename... SS>
  struct StateList<S, SS...>
  {
    static void reset()
    {
      _state_instance<S>::value = S();
      StateList<SS...>::reset();
    }
  };


  template <typename F>
  struct MooreMachine : tinyfsm::Fsm<F>
  {
    virtual void entry(void){}; 
    void exit(void){};          
  };

  template <typename F>
  struct MealyMachine : tinyfsm::Fsm<F>
  {
    void entry(void){}; 
    void exit(void){}; 
  };

} 

// 访问状态的实例 tinyfsm::Fsm<_FSM>::current_state_ptr      tinyfsm::_state_instance<_STATE>::value
#define FSM_INITIAL_STATE(_FSM, _STATE)                    \
  namespace tinyfsm                                        \
  {                                                        \
    template <>                                            \
    void Fsm<_FSM>::set_initial_state(void)                \
    {                                                      \
      current_state_ptr = &_state_instance<_STATE>::value; \
    }                                                      \
  }

#endif /* TINYFSM_HPP_INCLUDED */
