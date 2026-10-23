/**
 * @FilePath: ecat_soem_sys.h
 * @Author: AT
 * @Date: 2025-06-07 11:07:41
 * @LastEditTime: 2025-11-06 22:35:56
 * @Description:
 * @Copyright (c) 2025 by Balance Beam, All Rights Reserved.
 */
/** \file ecat_soem_sys.h
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
#ifndef __ECAT_SOEM_SYS_H_
#define __ECAT_SOEM_SYS_H_
#include "dev_servo.h"
#include "ecat_io.h"
#include "ecat_type_def.h"

/*Initialize Ethercat stack, initialize servo, io*/
bool ecat_soem_sys_init(char *ifname, int cycletime_us);

/*Change Ethercat operation state*/
bool ecat_soem_changeEcatState(long long slave, int state);

/*Get  ethercat master state*/
int ecat_sys_get_ethercat_master_state(void);
/*Get ethercat slave state*/
int ecat_sys_get_ethercat_slave_state(int slave_slot);
/*Get ethercat device number*/
int ecat_sys_get_ecat_nodes(void);
/*Get ethercat slave working counter*/
int ecat_sys_get_ethercat_slave_wc(int domain_group);

int ecat_soem_readstate(void);

#endif //__ECAT_SOEM_SYS_H_
