/** \file ecat_device_cfg.h
 * \brief Define all kinds of ethercat device pdo mapping and initializing.
 *
 * Usage : Export supported_ecat_devices[].
 *
 *
 *
 * 
 *
 * (c) AT  2025.01.12
 */

#ifndef __ECAT_PDO_MAPPING_H__
	#define __ECAT_PDO_MAPPING_H__

	#include <ecrt.h>
	
#ifndef SIZEOF
	#define SIZEOF(x) (sizeof(x)/sizeof(x[0]))
#endif

typedef struct {
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */
    uint8_t bit_length; /**< Size of the PDO entry in bit. */
	uint32_t value; /*Init value*/
} ec_init_info_t;


typedef struct
{
	unsigned int	vid;			/**/
	unsigned int 	pcode;
	unsigned int 	rno;			/*revision number*/
	char dc_sync[8];		// Switch of dc sync
	unsigned short assign_activate; // Assign active value
	ec_sync_info_t* p_sync_info;	// Sync info
	ec_init_info_t* p_init_info;	// Initialize info
} EcatDevMapDesc;


extern EcatDevMapDesc supported_ecat_devices[];

#endif