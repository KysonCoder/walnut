/** \file ecat_igh_config.h
 * \brief Configure ecat device pdo mapping.
 *
 * Usage :
 *
 *
 *
 *
 *
 * (c) AT  2025.01.12
 */
#ifndef __ECAT_CONFIG_H_
	#define __ECAT_CONFIG_H_


	#include "ecat_type_def.h"
	#include "fbbase_log.h"
	int ecat_igh_setup_pdo(EcatApp *ecat_app, uint16_t slave);
	bool ecat_igh_bind_data(EcatApp *ecat_app, PECATMASTER pEcatMaster);
    ec_pdo_entry_reg_t * ec_igh_register_bject(EcatApp *ecat_app, uint16_t slave, ec_pdo_entry_reg_t *pRetRegLst, ec_pdo_entry_reg_t **ppRetRegLst, ec_pdo_entry_reg_t *pRetRegLst_tx, ec_pdo_entry_reg_t **ppRetRegLst_tx);

	
#endif  // __ECAT_CONFIG_H_
