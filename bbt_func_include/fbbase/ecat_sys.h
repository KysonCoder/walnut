/**
 * @FilePath: ecat_sys.h
 * @Author: AT
 * @Date: 2025-06-07 11:07:41
 * @LastEditTime: 2026-01-25 22:02:47
 * @Description:
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */
/** \file ecat_sys.h
 * \brief Ethercat system access
 *
 * Usage : Use function to initialize SOEM and ethercat
 *      servo + ethercat IO
 *
 *
 *
 *
 * (c) AT  2022.02.02
 */
#ifndef __ECAT_SYS_H_
#define __ECAT_SYS_H_

// clang-format off
#include "ecrt.h"
#include "ecat_type_def.h"
// clang-format on
#include "dev_servo.h"
#include "ecat_io.h"
#include "fbbase_log.h"

using FBBASE::SERVO::ECAT_SERVO_SYSTEM;
using FBBASE::SERVO::PECAT_SERVO_SYSTEM;

#ifdef CONFIG_SUPPORT_ECAT_SOEM
#include "osal.h"
#endif
/****************************************************************************/
// Application parameters
#define FREQUENCY 1000
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B)                                                          \
  (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

typedef struct {
  ECATMASTER ecat_master;         // topology of ecathercat
  ECAT_SERVO_SYSTEM servo_system; // servo devices in the bus
  ECAT_IO_SYSTEM io_system;
  long long hd_kernel;
  long long hd_timer;          // timer handle
  long long hd_error_callback; // handle of error callback
  long long hd_top_callback;   // handle of topology callback
  long long hd_plc_callback;
  long long hd_dataset_callback;
  long long hd_timer_callback;
  long long start_time;
  int cycle_time;
  int shift_time;
  long long app_ptr;
  long long sys_ptr;
} ECATSYSTEM, *PECATSYSTEM;

/*Initialize Ethercat stack, initialize servo, io*/
bool ecat_sys_init(char *ifname, int cycletime_us);
/*Read data from Ethercat slave by sdo, can use complete access method*/
int readEcatDataObj(long long slave, int objIndex, int varIndex, void *pData,
                    int *pSize, bool ca = false);
/*Write data into Ethercat slave by sdo, can use complete access method*/
int writeEcatDataObj(long long slave, int objIndex, int varIndex,
                     const void *pData, int size, bool ca = false);
/*Read value from Ethercat mapped PDO entry address*/
int ecat_read_var(PECAT_DATA_VAR_INFO p);
/*Write value into Ethercat mapped PDO entry address*/
int ecat_write_var(PECAT_DATA_VAR_INFO p, int val);
/*Fine pdo entry var description by pdo*/
PECAT_DATA_VAR_INFO find_pdo_var_address_by_pdo(PECATSLAVE pSlave, int pdo_index,
                                         int var_index);
/*Fine pdo entry var description by sdo*/
PECAT_DATA_VAR_INFO find_pdo_var_address_by_sdo(PECATSLAVE pSlave, int sdo_index,
                                         int sdo_subindex);

/*Get  ethercat master state*/
int ecat_sys_get_ethercat_master_state(void);
/*Get ethercat slave state*/
int ecat_sys_get_ethercat_slave_state(int slave_slot);
/*Get ethercat device number*/
int ecat_sys_get_ecat_nodes(void);
/*Get ethercat slave working counter*/
int ecat_sys_get_ethercat_slave_wc(int domain_group);

void add_timespec(struct timespec *ts, int64_t addtime);

/*Get sdo/pdo address+bit offset+ bit length by SDO index + SDO subindex*/
bool ecat_sys_get_sdo_addr(int slave, int sdo_index, int sdo_subindex,
                           char *&sdoAddr, int &sdoBitOffset, int &sdoBitLen);

/*Get pdo mapping by slave handle*/
bool ecat_sys_get_pdo_cfg_local(long long slave, int pdo_index,
                                unsigned long *pcfg, int max_vars,
                                int *got_vars);

#endif //__ECAT_SYS_H_
