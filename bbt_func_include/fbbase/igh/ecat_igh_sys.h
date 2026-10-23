/** \file ecat_igh_sys.h
 * \brief Ethercat IGH system access
 *
 * Usage : Use function to initialize IGH and ethercat
 *      servo + ethercat IO
 *
 *
 *
 *
 * (c) AT  2025.01.12
 */
#ifndef __ECAT_IGH_SYS_H_
	#define __ECAT_IGH_SYS_H_
	
	/** Possible EtherCAT slave states */
	typedef enum
	{
	/** No valid state. */
	EC_STATE_NONE           = 0x00,
	/** Init state*/
	EC_STATE_INIT           = 0x01,
	/** Pre-operational. */
	EC_STATE_PRE_OP         = 0x02,
	/** Boot state*/
	EC_STATE_BOOT           = 0x03,
	/** Safe-operational. */
	EC_STATE_SAFE_OP        = 0x04,
	/** Operational */
	EC_STATE_OPERATIONAL    = 0x08,
	/** Error or ACK error */
	EC_STATE_ACK            = 0x10,
	EC_STATE_ERROR          = 0x10
	} ec_state;

	bool ecat_igh_sys_init(char* ifname, int scan_cycle, bool dc_sync);
	int ecat_igh_get_slave_state(int slave_pos, int * perror_flag , int * pal_state , char * pname, int name_size);
	bool ecat_igh_changeEcatState(long long slave, int state);
	int ecat_igh_readstate(void);
	bool ecat_igh_domain_state(void);
	
#endif //__ECAT_SYS_H_
