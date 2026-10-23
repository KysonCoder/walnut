/**
 * @FilePath: mit_gateway_servo.h
 * @Author: AT
 * @Date: 2026-04-09 21:35:42
 * @LastEditTime: 2026-04-09 22:17:56
 * @Description: Declare the gateway mit servo interface. The interface is
 * based on variants.
 * @Copyright (c) 2026 by Balance Beam, All Rights Reserved.
 */

#ifndef _MIT_GATEWAY_SERVO_H_
#define _MIT_GATEWAY_SERVO_H_
#include "fbvar.h"
#include "i_servo.h"
#include <string>
#include "fbbase_log.h"
namespace FBBASE {
namespace SERVO {


// Declare a new class based on classs IServo

// Declare a new class based on classs IServo

class MitGatewayServo : public IServo {
public:

// Define max number fbvar per gateway servo node
typedef enum {  
  NODE_VAR_INDEX_CTRL_WORD = 0,
  NODE_VAR_INDEX_OP_MODE,
  NODE_VAR_INDEX_TARGET_POS,
  NODE_VAR_INDEX_TARGET_SPEED,
  NODE_VAR_INDEX_TARGET_TORQUE,
  NODE_VAR_INDEX_SPEED_OFFSET,
  NODE_VAR_INDEX_TORQUE_OFFSET,
  NODE_VAR_INDEX_STATUS_WORD,
  NODE_VAR_INDEX_ACTUAL_POS,
  NODE_VAR_INDEX_ACTUAL_SPEED,
  NODE_VAR_INDEX_ACTUAL_TORQUE,
  NODE_VAR_INDEX_ERROR_CODE,
  NODE_VAR_INDEX_MAX
} SERVO_NODE_VAR_INDEX;

// Define max number of gateway servo nodes
#define MAX_SERVO_NODES 100

// Define cia servo drive control structure
typedef struct {
  std::string var_name[NODE_VAR_INDEX_MAX];
  FBVAR_HANDLE var_handles[NODE_VAR_INDEX_MAX];
  bool var_valid[NODE_VAR_INDEX_MAX];
} SERVO_NODE;

  MitGatewayServo();
  ~MitGatewayServo();

  // Initialize the servo based on variants
  bool init(const std::string mapping_cfg, const int scan_cycleus);

  // write general servo operation mode
  bool set_op_mode(int nodeIndex, int op_mode) override;
  // read general servo operation mode
  bool get_op_mode(int nodeIndex, int &op_mode) override;
  // write data to general servo
  bool set_forward_datas(int nodeIndex, int pos, int speed_offset = 0,
                         int torque_offset = 0, int target_speed = 0,
                         int target_torque = 0) override;
  //  read general servo feedback data
  bool get_measure_datas(int nodeIndex, int &status, int &pos, int &speed,
                         int &torque) override;

  /*Clear one servo fault*/
  bool clear_fault(int nodeIndex) override;
  /*Get general servo status word*/
  bool get_status_word(int nodeIndex, int &status) override;
  /*Get general servo error code*/
  bool get_error_code(int nodeIndex, int &error_code) override;
  /*Read general servo control word*/
  bool get_ctrl_word(int nodeIndex, int &word) override;
  /*Write value into general servo control word*/
  bool set_ctrl_word(int nodeIndex, int word) override;

  /*Enable general servo switch*/
  bool enable_switch(int nodeIndex) override;
  /*Enable general servo operation*/
  bool enable_operation(int nodeIndex) override;
  /*Clear ecat all servo node fault*/
  bool clear_all_fault(void) override;
  /*Check ecat node running or operation status */
  bool check_operation_status(int nodeIndex, bool &status) override;
  /*Check ecat node power status */
  bool check_power_status(int nodeIndex, bool &status) override;
  /*Check ecat node fault status */
  bool check_fault_status(int nodeIndex, bool &status) override;
  /*Disable general servo switch*/
  bool disable_switch(int nodeIndex) override;
  /*Disable general servo operation*/
  bool disable_operation(int nodeIndex) override;
  /*Get general servo nodes*/
  bool get_nodes(int &totalNodes) override;
  /*Get actual position   to general servo*/
  bool get_actual_position(int nodeIndex, int &postion) override;
  /*Read  reference position written to general servo*/
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

private:
  // Check nodeIndex validity
  bool check_nodeIndex(int nodeIndex) {
    if (nodeIndex >= 0 && nodeIndex < nodes)
      return true;
    else
      return false;
  }

  // Increase the number of nodes
  bool add_node(int &nodeIndex) {

    if (MAX_SERVO_NODES > nodes) {
      nodes++;
      nodeIndex = nodes - 1;
      FB_I("add_node: nodeIndex=%d nodes=%d ", nodeIndex, nodes);
      return true;
    } else
      return false;
  }

  // Check class initialized
  bool check_init() {
    if (initialized)
      return true;
    else
      return false;
  }

  // Check var handle
  bool check_varHandle(FBVAR_HANDLE handle) {
    if (handle != NULL)
      return true;
    else
      return false;
  }

  // Define class initialized
  bool initialized;
  // Define all actuve CiA402 gateway servo nodes
  int nodes;

  // Define the bus scan time
  int scan_cycleus;

  /*
    NODE_VAR_INDEX_CTRL_WORD = 0,
NODE_VAR_INDEX_CTRL_WORD = 0,
  NODE_VAR_INDEX_OP_MODE,
  NODE_VAR_INDEX_TARGET_POS,
  NODE_VAR_INDEX_TARGET_SPEED,
  NODE_VAR_INDEX_TARGET_TORQUE,
  NODE_VAR_INDEX_SPEED_OFFSET,
  NODE_VAR_INDEX_TORQUE_OFFSET,
  NODE_VAR_INDEX_STATUS_WORD,
  NODE_VAR_INDEX_ACTUAL_POS,
  NODE_VAR_INDEX_ACTUAL_SPEED,
  NODE_VAR_INDEX_ACTUAL_TORQUE,
  NODE_VAR_INDEX_ERROR_CODE,
   */

  // Check and set the cia402 gateway servo var name
  inline bool check_servonode_varname(SERVO_NODE &servoNode,
                                      SERVO_NODE_VAR_INDEX varIndex,
                                      std::string prefixStr,
                                      std::string suffixStr) {
    if ("" != prefixStr && "" != suffixStr) {
      int value = 0;
      if (fbvar_get(prefixStr + suffixStr, value)) {
        servoNode.var_name[varIndex] = prefixStr + suffixStr;
        servoNode.var_valid[varIndex] = true;
        return true;
      }
    }
    return false;
  }

  // Define all CiA402 gateway servo nodes
  SERVO_NODE servo_nodes[MAX_SERVO_NODES];
};
} // namespace SERVO
} // namespace FBBASE
#endif // _CIA402_GATEWAY_SERVO_H_
