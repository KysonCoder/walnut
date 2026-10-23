/** \file ecat_config.h
 * \brief Configure ecat device pdo mapping.
 *
 * Usage :
 *
 *
 *
 *
 *
 * (c) AT  2022.02.03
 */
#ifndef __ECAT_CONFIG_H_
	#define __ECAT_CONFIG_H_
	#include <ecrt.h>
	#include <ecat_type_def.h>
	#include "fbbase_log.h"
	bool ecat_soem_bind_data(PECATMASTER pEcatMaster);
	int ecat_soem_setup_pdo(uint16 slave);
#endif  // __ECAT_CONFIG_H_
