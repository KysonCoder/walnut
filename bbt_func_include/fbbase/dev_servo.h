/**
 * @FilePath: dev_servo.h
 * @Author: AT
 * @Date: 2025-10-28 21:35:23
 * @LastEditTime: 2025-12-13 12:12:37
 * @Description: Declare general servo control method.
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */
#ifndef _DEV_SERVO_H_
#define _DEV_SERVO_H_
#include <iostream>
#include <mutex>

#include "cia402_ecat_servo.h"
#include "cia402_gateway_servo.h"
#include "mit_gateway_servo.h"
#include "i_servo.h"
#include "fbbase_log.h"

namespace FBBASE {
namespace SERVO {

#define MAX_SERVOS 400
#define CIA402_GATEWAY_SERVO_START_INDEX 200
#define MIT_GATEWAY_SERVO_START_INDEX 300
typedef struct {
  IServo *pServo;
  int internalIndex;
  bool valid;
} SERVOCTRL;

// Define class DEVSERVO
class DEVSERVO : public IServo {
public:
  Cia402GatewayServo cia402_gateway_servo;
  Cia402EcatServo cia402_ecat_servo;
  MitGatewayServo mit_gateway_servo;

public:
  DEVSERVO(const DEVSERVO &) = delete;
  DEVSERVO &operator=(const DEVSERVO &) = delete;

  static DEVSERVO *getInstance() {
    if (instance == nullptr) {
      std::lock_guard<std::mutex> lock(mtx);
      if (instance == nullptr) {
        instance = new DEVSERVO();
        for (int i = 0; i < MAX_SERVOS; i++) {
          instance->servos_array[i].pServo = NULL;
          instance->servos_array[i].internalIndex = 0;
          instance->servos_array[i].valid = false;
        }
        instance->servos_num = 0;
      }
    }
    return instance;
  }

  void init(const std::string ecat_servo_cfg, int cycletime_us);

  // write  servo operation mode
  bool set_op_mode(int nodeIndex, int op_mode) override;
  // read  servo operation mode
  bool get_op_mode(int nodeIndex, int &op_mode) override;
  // write data to  servo
  bool set_forward_datas(int nodeIndex, int pos, int speed_offset = 0,
                         int torque_offset = 0, int target_speed = 0,
                         int target_torque = 0) override;
  //  read  servo feedback data
  bool get_measure_datas(int nodeIndex, int &status, int &pos, int &speed,
                         int &torque) override;

  /*Clear one servo fault*/
  bool clear_fault(int nodeIndex) override;
  /*Get  servo status word*/
  bool get_status_word(int nodeIndex, int &status) override;
  /*Get  servo error code*/
  bool get_error_code(int nodeIndex, int &error_code) override;
  /*Read  servo control word*/
  bool get_ctrl_word(int nodeIndex, int &word) override;
  /*Write value into  servo control word*/
  bool set_ctrl_word(int nodeIndex, int word) override;

  /*Enable  servo switch*/
  bool enable_switch(int nodeIndex) override;
  /*Enable  servo operation*/
  bool enable_operation(int nodeIndex) override;
  /*Clear  all servo node fault*/
  bool clear_all_fault(void) override;
  /*Check  node running or operation status */
  bool check_operation_status(int nodeIndex, bool &status) override;
  /*Check  node power status */
  bool check_power_status(int nodeIndex, bool &status) override;
  /*Check  node fault status */
  bool check_fault_status(int nodeIndex, bool &status) override;
  /*Disable  servo switch*/
  bool disable_switch(int nodeIndex) override;
  /*Disable  servo operation*/
  bool disable_operation(int nodeIndex) override;
  /*Get general servo nodes*/
  bool get_nodes(int &totalNodes) override;
  /*Get actual position  to  servo*/
  bool get_actual_position(int nodeIndex, int &postion) override;
  /*Get refernece position  written to  servo*/
  bool get_reference_position(int nodeIndex, int &postion) override;
  /*Write gateway servo position*/
  bool set_position(int nodeIndex, int postion) override;
  /*Write gateway servo speed offset*/
  bool set_speed_offset(int nodeIndex, int speed) override;
  /*Write gateway servo torque offset*/
  bool set_torque_offset(int nodeIndex, int torque) override;
  /*Write gateway servo target speed*/
  bool set_target_speed(int nodeIndex, int speed) override;
  /*Write gateway servo target torque*/
  bool set_target_torque(int nodeIndex, int torque) override;
  /*Servo data exchange with ecat stack*/
  bool data_exchange(void) override;

  /*Get servo pos encoder pulses per round*/
  bool get_pos_encoder_res(int nodeIndex, int &pulses) override;
  /*Get servo rated torque*/
  bool get_rated_torque(int nodeIndex, double &torque) override;
  /*Check new reference*/
  bool check_new_reference(int nodeIndex, int newPos);

private:
  bool check_int32_minus_overflow(long int va, long int vb);
  bool check_nodeIndex(int nodeIndex) {
    if (nodeIndex >= 0 && nodeIndex < MAX_SERVOS) {
      if (NULL != servos_array[nodeIndex].pServo) {
        return true;
      }
    }
    FB_E("check_nodeIndex: ERROR! nodeIndex=%d pServo=0x%llx", nodeIndex,
           (long long)servos_array[nodeIndex].pServo);
    return false;
  }
  bool check_init() {

    if (initialized) {
      return true;

    } else {
      FB_W("DEVSERVO not initialized.");
      return false;
    }
  }
  bool registerServo(IServo &servo, int startIndex);

  static DEVSERVO *instance;
  static std::mutex mtx;
  static bool initialized;

  DEVSERVO() { FB_I("DEVSERVO instance created."); }

private:
  SERVOCTRL servos_array[MAX_SERVOS];

  int servos_num; // Number of servos
};

/*EtherCAT Servo data exchange with ecat stack*/
inline bool ecat_servo_data_exchange(void) {
  return FBBASE::SERVO::DEVSERVO::getInstance()
      ->cia402_ecat_servo.data_exchange();
}

/*EtherCAT gateway Servo data exchange with ecat stack*/
inline bool ecat_gateway_servo_data_exchange(void) {
  return FBBASE::SERVO::DEVSERVO::getInstance()
      ->cia402_gateway_servo.data_exchange();
}
// Initialize ecat servo
inline bool dev_servo_init(char *pServoConf, int cycletime_us) {
  const std::string cfg(pServoConf);
  FBBASE::SERVO::DEVSERVO::getInstance()->init((const std::string)cfg,
                                               cycletime_us);
  return true;
}

inline bool dev_servo_get_nodes(int &totalNodes) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->get_nodes(totalNodes);
}

inline bool dev_servo_disable_switch(int nodeIndex) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->disable_switch(nodeIndex);
}

inline bool dev_servo_enable_switch(int nodeIndex) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->enable_switch(nodeIndex);
}

inline bool dev_servo_disable_operation(int nodeIndex) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->disable_operation(nodeIndex);
}

inline bool dev_servo_enable_operation(int nodeIndex) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->enable_operation(nodeIndex);
}

inline bool dev_servo_read_measure_data(int nodeIndex, int &status, int &pos,
                                        int &speed, int &torque) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->get_measure_datas(
      nodeIndex, status, pos, speed, torque);
}

inline bool dev_servo_write_forward_data(int nodeIndex, int pos,
                                         int speed_offset, int torque_offset,
                                         int target_speed, int target_torque) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->set_forward_datas(
      nodeIndex, pos, speed_offset, torque_offset, target_speed, target_torque);
}

inline bool dev_servo_write_forward_data_safely(int nodeIndex, int pos,
                                                int speed_offset,
                                                int torque_offset,
                                                int target_speed,
                                                int target_torque) {
  if (true == FBBASE::SERVO::DEVSERVO::getInstance()->check_new_reference(
                  nodeIndex, pos)) {
    return FBBASE::SERVO::DEVSERVO::getInstance()->set_forward_datas(
        nodeIndex, pos, speed_offset, torque_offset, target_speed,
        target_torque);
  } else {
    return false;
  }
}

inline bool dev_servo_get_pos_encoder_res(int nodeIndex, int &encoder_res) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->get_pos_encoder_res(
      nodeIndex, encoder_res);
}

inline bool dev_servo_get_rated_torque(int nodeIndex, double &rated_torque) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->get_rated_torque(nodeIndex,
                                                                  rated_torque);
}

inline bool dev_servo_set_op_mode(int nodeIndex, int opMode) {
  return FBBASE::SERVO::DEVSERVO::getInstance()->set_op_mode(nodeIndex, opMode);
}

} // namespace SERVO
} // namespace FBBASE

#endif // _DEV_SERVO_H_