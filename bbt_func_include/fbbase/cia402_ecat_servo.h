/**
 * @FilePath: cia402_ecat_servo.h
 * @Author: AT
 * @Date: 2025-11-01 16:06:31
 * @LastEditTime: 2025-11-23 09:42:18
 * @Description: Define cia402 EtherCAT servo methods.
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */
#ifndef _CIA402_ECAT_SERVO_H_
#define _CIA402_ECAT_SERVO_H_

// clang-format off
#include "ecrt.h"
#include "ecat_type_def.h"
// clang-format on

#include "fbvar.h"
#include "i_servo.h"
#include "fbbase_log.h"
#include <string>

namespace FBBASE {
namespace SERVO {

#pragma pack(push, 4)

typedef struct {
  int vendor;
  int product;
  // Non-cyclic data
  // device object index, inputs to servo node
  unsigned int obj_index_quick_stop;
  unsigned int obj_subindex_quick_stop;

  unsigned int obj_index_ctrl_word;
  unsigned int obj_subindex_ctrl_word;

  unsigned int obj_index_operation_mode;
  unsigned int obj_subindex_operation_mode;

  unsigned int obj_index_target_position;
  unsigned int obj_subindex_target_position;

  unsigned int obj_index_target_speed;
  unsigned int obj_subindex_target_speed;

  unsigned int obj_index_target_torque;
  unsigned int obj_subindex_target_torque;

  unsigned int obj_index_target_kp;
  unsigned int obj_subindex_target_kp;

  unsigned int obj_index_target_kv;
  unsigned int obj_subindex_target_kv;

  unsigned int obj_index_target_ki;
  unsigned int obj_subindex_target_ki;

  // device object index, outputs from servo node
  unsigned int obj_index_error_code;
  unsigned int obj_subindex_error_code;

  unsigned int obj_index_status_word;
  unsigned int obj_subindex_status_word;

  unsigned int obj_index_actual_position;
  unsigned int obj_subindex_actual_position;

  unsigned int obj_index_actual_speed;
  unsigned int obj_subindex_actual_speed;

  unsigned int obj_index_actual_torque;
  unsigned int obj_subindex_actual_torque;

  unsigned int obj_index_actual_kp;
  unsigned int obj_subindex_actual_kp;

  unsigned int obj_index_actual_kv;
  unsigned int obj_subindex_actual_kv;

  unsigned int obj_index_actual_ki;
  unsigned int obj_subindex_actual_ki;

  unsigned int obj_index_pos_encoder_res;
  unsigned int obj_subindex_pos_encoder_res;

  unsigned int obj_index_vel_encoder_res;
  unsigned int obj_subindex_vel_encoder_res;

  unsigned int obj_index_gear_ratio;
  unsigned int obj_subindex_gear_ratio;

  unsigned int obj_index_rated_torque;
  unsigned int obj_subindex_rated_torque;

  // device object index, RW
  unsigned int obj_index_position_offset;
  unsigned int obj_subindex_position_offset;

  unsigned int obj_index_speed_offset;
  unsigned int obj_subindex_speed_offset;

  unsigned int obj_index_torque_offset;
  unsigned int obj_subindex_torque_offset;

  // Cyclic data, process data object
  // inputs
  unsigned int pdo_index_ctrl_word;
  unsigned int pdo_subindex_ctrl_word;
  unsigned int pdo_index_target_position;
  unsigned int pdo_subindex_target_position;
  unsigned int pdo_index_offset_speed; // offset speed
  unsigned int pdo_subindex_target_speed;
  unsigned int pdo_index_offset_torque; // offset torque
  unsigned int pdo_subindex_target_torque;
  unsigned int pdo_index_target_kp;
  unsigned int pdo_subindex_target_kp;
  unsigned int pdo_index_target_kv;
  unsigned int pdo_subindex_target_kv;
  unsigned int pdo_index_target_ki;
  unsigned int pdo_subindex_target_ki;

  // outputs
  unsigned int pdo_index_status_word;
  unsigned int pdo_subindex_status_word;
  unsigned int pdo_index_actual_position;
  unsigned int pdo_subindex_actual_position;
  unsigned int pdo_index_actual_speed;
  unsigned int pdo_subindex_actual_speed;
  unsigned int pdo_index_actual_torque;
  unsigned int pdo_subindex_actual_torque;
  unsigned int pdo_index_actual_kp;
  unsigned int pdo_subindex_actual_kp;
  unsigned int pdo_index_actual_kv;
  unsigned int pdo_subindex_actual_kv;
  unsigned int pdo_index_actual_ki;
  unsigned int pdo_subindex_actual_ki;
  // Unit transfer
  int servo_speed_unit_factor; // Unit transfer: controller speed(inc/s) *
                               // servo_speed_unit_factor = servo speed
  int servo_kp_unit_factor;    // Unit transfer: controller kp *
                               // servo_kp_unit_factor = servo kp
  int servo_kv_unit_factor;    // Unit transfer: controller kv *
                               // servo_kv_unit_factor = servo kv
  int servo_ki_unit_factor;    // Unit transfer: controller ki *
                               // servo_ki_unit_factor = servo ki
  int servo_ti_unit_factor;    // Unit transfer: controller ti(s) *
                               // servo_ti_unit_factor = servo ti
  bool servo_use_ti; // Velocity loop control use Ti to control velocity by
                     // ECAT, or ki.

  // Non-cyclic data
  // device object index, inputs to servo node
  unsigned int obj_index_single_turn;    // Encoder single turn index
  unsigned int obj_subindex_single_turn; // Encoder single turn sub index
  unsigned int obj_index_multi_turn;     // Encoder multi turn index
  unsigned int obj_subindex_multi_turn;  // Encoder multi turn sub index

  unsigned int obj_index_dc_voltage;    // DC link voltage index
  unsigned int obj_subindex_dc_voltage; // DC link voltage  sub index

  // Cyclic data, process data object
  // inputs --- Continued
  unsigned int rxpdo_index_operation_mode;
  unsigned int rxpdo_subindex_operation_mode;
  unsigned int rxpdo_index_target_speed;
  unsigned int rxpdo_subindex_target_speed;
  unsigned int rxpdo_index_target_torque;
  unsigned int rxpdo_subindex_target_torque;

  // Non-cyclic data
  // device object index, inputs to servo node
  unsigned int obj_index_offset_speed; // same as obj_index_speed_offset
  unsigned int obj_subindex_offset_speed;

  unsigned int obj_index_offset_torque;
  unsigned int obj_subindex_offset_torque;

  unsigned int obj_index_firmware_version; // firmware version
  unsigned int obj_subindex_firmware_version;

  unsigned int obj_index_hardware_version; // hardware version
  unsigned int obj_subindex_hardware_version;

  unsigned int obj_index_following_err;
  unsigned int obj_subindex_following_err;
  unsigned int txpdo_index_following_err;
  unsigned int txpdo_subindex_following_err;

  unsigned int txpdo_index_error_code;    // servo error code pdo index
  unsigned int txpdo_subindex_error_code; // servo error code pdo sub-index

} ECAT_SERVO_NODE_CONFIG, *PECAT_SERVO_NODE_CONFIG;

/* pdo data in servo node */
typedef struct {
  ECAT_DATA_VAR_INFO ctrl_word;
  ECAT_DATA_VAR_INFO status_word;
  ECAT_DATA_VAR_INFO position;
  ECAT_DATA_VAR_INFO speed;  // offset speed
  ECAT_DATA_VAR_INFO torque; // offset torque
  ECAT_DATA_VAR_INFO kp;
  ECAT_DATA_VAR_INFO kv;
  ECAT_DATA_VAR_INFO ki;
  ECAT_DATA_VAR_INFO valid_flag;
  ECAT_DATA_VAR_INFO op_mode;       // Operation mode : CSP(8), CSV(9), CST(10)
  ECAT_DATA_VAR_INFO tgt_speed;     // target speed in CSV mode
  ECAT_DATA_VAR_INFO tgt_torque;    // Target torque in CST mode
  ECAT_DATA_VAR_INFO following_err; // Following error
  ECAT_DATA_VAR_INFO error_code;    // Error code

} ECAT_SERVO_NODE_PDO_DATA, *PECAT_SERVO_NODE_PDO_DATA;

typedef struct {
  unsigned int index_operation_mode; // 0x6060

} ECAT_SERVO_NODE_CTRL_PARAM, *PECAT_SERVO_NODE_CTRL_PARAM;

typedef struct {
  long long hd_node; // handle of slave
  int node_position; /*position in ecat bus */
  ECAT_SERVO_NODE_CONFIG node_config;
  ECAT_SERVO_NODE_PDO_DATA measure_data;
  ECAT_SERVO_NODE_PDO_DATA forward_data;
  unsigned int pos_encoder_res;
  double rated_torque;
  unsigned short val_ctr_word;
  int val_position;
  int val_speed;
  int val_torque;
  int val_kp;
  int val_kv;
  int val_ki;
  int val_op_mode;
  int val_tgt_speed;
  int val_tgt_torque;
  int delta;          // test purpose
  char fwversion[64]; // Firmware version
  char hwversion[64]; // Hardware version
} ECAT_SERVO_NODE, *PECAT_SERVO_NODE;

typedef struct {
  int num_of_servo_nodes;
  ECAT_SERVO_NODE ecat_servo_nodes[MAX_NUM_OF_NODES_PER_BOARD];
  // SENSOR_MGR_SYNC_PARAMETERS sensor_mgr_sync_parameters;
} ECAT_SERVO_BOARD, *PECAT_SERVO_BOARD;

typedef enum {
  control_mode_none,
  control_mode_position,
  control_mode_speed,
  control_mode_torque
} SERVO_CTRL_MODE;

typedef enum {
  servo_status_unknown,
  servo_status_error,
  servo_status_inited,
  servo_status_enabled,
  servo_status_disabled
} SERVO_STATUS;

typedef struct {
  long long hd_master;
  long long hd_dataset;
  int num_of_ecat_boards;
  ECAT_SERVO_BOARD eact_servo_boards[MAX_NUM_OF_BOARDS];
  SERVO_STATUS servo_status;
  SERVO_CTRL_MODE servo_control_mode;
  void *app_ptr;
  void *sys_ptr;
  char valid;
} ECAT_SERVO_SYSTEM, *PECAT_SERVO_SYSTEM;

class Cia402EcatServo : public IServo {
public:
  Cia402EcatServo() {};
  ~Cia402EcatServo() {};

  bool prepare_enable(void);
  bool disable_all(void);
  // Initialize ecat servo
  bool init(const std::string pServoConf);

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
  bool check_status(int &status);
  /*Disable general servo switch*/
  bool disable_switch(int nodeIndex) override;
  /*Disable general servo operation*/
  bool disable_operation(int nodeIndex) override;
  /*Get general servo nodes*/
  bool get_nodes(int &totalNodes) override;
  /*Servo data exchange with ecat stack*/
  bool data_exchange(void) override;
  /*Read position reference written to general servo*/
  bool get_actual_position(int nodeIndex, int &postion) override;
  /*Read  reference position written to general servo*/
  bool get_reference_position(int nodeIndex, int &postion) override;
  /*Write ecat servo position*/
  bool set_position(int nodeIndex, int postion) override;
  /*Write ecat servo speed offset*/
  bool set_speed_offset(int nodeIndex, int speed) override;
  /*Write ecat servo torque offset*/
  bool set_torque_offset(int nodeIndex, int torque) override;
  /*Write ecat servo target speed*/
  bool set_target_speed(int nodeIndex, int speed) override;
  /*Write ecat servo target torque*/
  bool set_target_torque(int nodeIndex, int torque) override;

  /*Get servo pos encoder pulses per round*/
  bool get_pos_encoder_res(int nodeIndex, int &pulses) override;
  /*Get servo rated torque*/
  bool get_rated_torque(int nodeIndex, double &torque) override;

private:
  bool read_pdo_cfg(unsigned long long hd_node, int pdo_index,
                    unsigned long *pcfg, int max_vars, int *got_vars);
  bool lookfor_var_in_pdo(unsigned long *pcfg, int actual_vars, int sdo_index,
                          int sdo_subindex, int &index);
  const PECAT_SERVO_NODE_CONFIG find_servo_node_config(PECATSLAVE p,
                                                       int *pstartindex);
  bool add_node(PECAT_SERVO_SYSTEM pServoSys, PECATSLAVE pSlave,
                PECAT_SERVO_NODE_CONFIG pConfig, int index, int slave_index);
  bool build_from_top(PECAT_SERVO_SYSTEM pServoSys, PECATMASTER pMaster);

  // Check nodeIndex validity
  bool check_nodeIndex(int nodeIndex) {
    if (nodeIndex >= 0 && nodeIndex < nodes)
      return true;
    else
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
  // Define all actuve CiA402 ecat servo nodes
  int nodes;

  // Define the bus scan time
  int scan_cycleus;
};

#pragma pack(pop)

} // namespace SERVO
} // namespace FBBASE

#endif // _CIA402_ECAT_SERVO_H_
