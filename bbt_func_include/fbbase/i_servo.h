/**
 * @FilePath: i_servo.h
 * @Author: AT
 * @Date: 2025-10-25 17:28:43
 * @LastEditTime: 2026-04-04 10:07:34
 * @Description: Define general servo interface for one kind field bus.
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */
#ifndef _I_SERVO_H_
#define _I_SERVO_H_

namespace FBBASE {
namespace SERVO {

#define DEFAULT_MOTOR_RATED_TORQUE 0.318 // Unit: N.M
#define DEFAULT_MOTOR_TORQUE_LIMIT 50    // Unit:  rated torque* 1/1000

#define MAX_NUM_OF_BOARDS 4
#define MAX_NUM_OF_NODES_PER_BOARD 60

#define ECAT_SERVO_OPERATION_CONTROL // Switch for ethercat servo (CIA402)
                                     // operation enable signal

/*new step eMObotic C servo parameters */
/*
RxPDO 0x1600
sub index --- value --- byte --- description
        0			3		1			number
of mapped objects 1	   0x60400010   2			control word 2
0x607a0020   4           target position 3      0x60ff0020   4
target speed
*/

/*
TxPDO 0x1a00
sub index --- value --- byte --- description
        0			3		1			number
of mapped objects 1      0x60410010	2			status word 2
0x60640020   4			position actual value 3      0x606c0020   4
speed actual value
*/
/*control word and command */
/*
6040h	Control word			unsigned16	RW
6041h	Status word				unsigned16	RO
605Ah	Quickstop option code	unsigned16	RW
605Bh	Shutdown option code	unsigned 16 RW
605Ch	Disable operation option code unsigned 16 RW
605Dh	Halt option code		unsigned 16 RW
605Eh	Fault reaction option code unsigned 16 RW
6060h	Modes of operation		INTEGER8	RW
6061h	Modes of operation display INTEGER8 RO
*/

/* 6 operation mode 0x6060 */
/*
 0 - mode not set
 1 - profile position mode
 3 - profile speed mode
 4 - profile torque mode
 6 - homing mode
 7 - interpolation position mode
 8 - cyclic synchronization position mode
*/

/*------------------------- Private Typedef -----------------------*/

// Verdor ID
#define ECAT_VENDOR_ID_QNDC 0x00000748
#define ECAT_VENDOR_ID_STONE 0x00418108
#define ECAT_VENDOR_ID_ZEERO 0x5A65726F
#define ECAT_VENDOR_ID_IPOS 0x1a3
#define ECAT_VENDOR_ID_TECHSERVO 0xC48
#define ECAT_VENDOR_ID_XMC 0x0000034E
#define ECAT_VENDOR_ID_INOVANCE 0x00100000
#define ECAT_VENDOR_ID_HUACHUANG 0x00009434

#define QNDC_SERVO_ERROR_CODE_UNDER_VOLTAGE 0x3220
#define QNDC_SERVO_ERROR_STO1_TRIGGERED 0xFF1D
#define QNDC_SERVO_ERROR_STO2_TRIGGERED 0xFF27

#define GENE_SERVO_ERROR_CODE_UNDER_VOLTAGE 0x3220
#define JHLAB_SERVO_ERROR_CODE_UNDER_VOLTAGE 0x3220

// CiA402 standard servo status word mask
typedef enum {
  SERVO_MASK_STAS_READ_TO_SWITCHED_ON = 0x0001,
  SERVO_MASK_STAS_SWITCHED_ON = 0x0002,
  SERVO_MASK_STAS_OPERATION_ENABLE = 0x0004,
  SERVO_MASK_STAS_FAULT = 0x0008,
  SERVO_MASK_STAS_VOLTAGE_ENABLE = 0x0010,
  SERVO_MASK_STAS_QUICK_STOP = 0x0020,
  SERVO_MASK_STAS_SWITCHED_ON_DISABLE = 0x0040,
  SERVO_MASK_STAS_WARNING = 0x0080,
  SERVO_MASK_STAS_MANUFACTURER_SPEC1 = 0x0100,
  SERVO_MASK_STAS_REMOTE = 0x0200,
  SERVO_MASK_STAS_TARGET_REACHED = 0x0400,
  SERVO_MASK_STAS_INTERNAL_LIMIT_ACTIVE = 0x0800,
  SERVO_MASK_STAS_OPERATION_MODE_SPEC = 0x0300,
  SERVO_MASK_STAS_MANUFACTURER_SPEC2 = 0xC000,
} CIA402_STATUS_WORD;
// CiA402 standard servo control word mask

/*
Bits:   15~11                  10~9             8       7
6~4  		3 	      	2     		1      		0 Manufacturer
Reserved      halt    Fault     	Operation      Enable      Quick Enable
Switch specific                                              Reset mode
operation  stop        Voltage		On
*/

typedef enum {
  SERVO_MASK_CTRL_SWITCHED_ON = 0x0001,
  SERVO_MASK_CTRL_VOLTAGE_ENABLE = 0x0002,
  SERVO_MASK_CTRL_QUICK_STOP = 0x0004, /*0, stop; 1, not stop*/
  SERVO_MASK_CTRL_OPERATION_ENABLE = 0x0008,
  SERVO_MASK_CTRL_RESET_FAULT = 0x0080
} CIA402_CTRL_WORD;

#define QNDC_SERVO_MASK_CTRL_RESET_MULTITURN                                   \
  (1 << 12) /*Rise edge to clear multi turn*/

typedef enum {
  mode_not_set = 0,
  mode_profile_position = 1,
  mode_profile_speed = 3,
  mode_profile_toque = 4,
  mode_homing = 6,
  mode_interpolation_position = 7,
  mode_cyclic_sync_position = 8,
  mode_cyclic_sync_speed = 9,
  mode_cyclic_sync_torque = 10
} CIA402_OP_MODE;

// Define a virtual servo interface class
class IServo {
public:
  // set general servo operation mode
  virtual bool set_op_mode(int nodeIndex, int op_mode) = 0;
  // get general servo operation mode
  virtual bool get_op_mode(int nodeIndex, int &op_mode) = 0;
  // set data to general servo
  virtual bool set_forward_datas(int nodeIndex, int pos, int speed_offset = 0,
                                 int torque_offset = 0, int target_speed = 0,
                                 int target_torque = 0) = 0;
  //  get general servo feedback data
  virtual bool get_measure_datas(int nodeIndex, int &status, int &pos,
                                 int &speed, int &torque) = 0;

  /*Clear one servo fault*/
  virtual bool clear_fault(int nodeIndex) = 0;
  /*Get general servo status word*/
  virtual bool get_status_word(int nodeIndex, int &status) = 0;
  /*Get general servo error code*/
  virtual bool get_error_code(int nodeIndex, int &error_code) = 0;
  /*Read general servo control word*/
  virtual bool get_ctrl_word(int nodeIndex, int &word) = 0;
  /*Set value into general servo control word*/
  virtual bool set_ctrl_word(int nodeIndex, int word) = 0;

  /*Enable general servo switch*/
  virtual bool enable_switch(int nodeIndex) = 0;
  /*Enable general servo operation*/
  virtual bool enable_operation(int nodeIndex) = 0;
  /*Clear ecat all servo node fault*/
  virtual bool clear_all_fault(void) = 0;
  /*Check ecat node running or operation status */
  virtual bool check_operation_status(int nodeIndex, bool &status) = 0;
  /*Check ecat node power status */
  virtual bool check_power_status(int nodeIndex, bool &status) = 0;
  /*Check ecat node fault status */
  virtual bool check_fault_status(int nodeIndex, bool &status) = 0;
  /*Disable general servo switch*/
  virtual bool disable_switch(int nodeIndex) = 0;
  /*Disable general servo operation*/
  virtual bool disable_operation(int nodeIndex) = 0;
  /*Get general servo nodes*/
  virtual bool get_nodes(int &totalNodes) = 0;

  /*Get actual position  written to general servo*/
  virtual bool get_actual_position(int nodeIndex, int &postion) = 0;
  /*Get reference position  written to general servo*/
  virtual bool get_reference_position(int nodeIndex, int &postion) = 0;
  /*Set gateway servo position*/
  virtual bool set_position(int nodeIndex, int postion) = 0;
  /*Set gateway servo speed offset*/
  virtual bool set_speed_offset(int nodeIndex, int speed) = 0;
  /*Set gateway servo torque offset*/
  virtual bool set_torque_offset(int nodeIndex, int torque) = 0;
  /*Set gateway servo target speed*/
  virtual bool set_target_speed(int nodeIndex, int speed) = 0;
  /*Set gateway servo target torque*/
  virtual bool set_target_torque(int nodeIndex, int torque) = 0;
  /*Exchange datas between field bus and buffer*/
  virtual bool data_exchange(void) = 0;
  /*Get servo pos encoder pulses per round*/
  virtual bool get_pos_encoder_res(int nodeIndex, int &pulses) = 0;
  /*Get servo rated torque*/
  virtual bool get_rated_torque(int nodeIndex, double &torque) = 0;
};

} // namespace SERVO
} // namespace FBBASE

#endif // _I_SERVO_H_