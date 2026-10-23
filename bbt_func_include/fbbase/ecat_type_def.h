/*
 * ecat_type_def.h
 *
 *  AT @ 2022.02.03
 */

#ifndef __ECAT_TYPE_DEF_H_
	#define __ECAT_TYPE_DEF_H_



#pragma pack(push, 4)



#define STONE_VENDOR_ID			0x00000748
#define STONE_SERVO_PID			0x00000000


/* type of device (slave) parameter from config file */
typedef struct 
{
	unsigned int vid;
	unsigned int pid;
	unsigned int slot_id;
	unsigned int nActDC;
	void *master_para;
	int dev_numbers;		/*driver supported dev numbers*/
	int unit_no_count;
	int unit_no_array[4];
	int dev_type;
	void *dev_data;			/*private data for driver.  for ec servo, it is the sm info.*/
} ec_driver_pdata;


typedef struct {
  int obj_type;
  char name[32];
  int data_type;
  int byte_offset;
  int bit_offset;
  int bit_length;
  int index;
  int sub_index;
  char valid; //0: invalid; 1: valid
} EC_DATA_VAR_INFO, *PEC_DATA_VAR_INFO;

typedef struct
{
	/* data area to be set before initialization */
	char name[32];
	unsigned short pdo_index; //from slave point of view: 0x1600 - RxPDO, 0x1a00 - TxPDO
	unsigned short pdo_sub_index;
	int pdo_total_bit_length;		//Total bit length with all vars; 
	/* data area to be filled after initialization */
 	int byte_offset;	
	int bit_offset;
	int bit_length;	
	int var_count; //entry num
	EC_DATA_VAR_INFO vars[64];
}EC_PDO_OBJECT, *PEC_PDO_OBJECT;

typedef struct
{
	unsigned int	vendor_id;
	unsigned int product_code;

	void *dev_data;
} EcatDevDesc;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#define ECAT_STATE_INIT                      0x00000001
#define ECAT_STATE_PREOP                   0x00000002
#define ECAT_STATE_BOOT                    0x00000003
#define ECAT_STATE_SAFEOP                 0x00000004
#define ECAT_STATE_OP                        0x00000008


#define MAX_NUM_OF_DATA_OBJECTS 20 //PDO NUM PER SLAVE
#define MAX_NUM_OF_SLAVES 64
#define MAX_LEN_OF_ECAT_NAME 32
#define MAX_NUM_OF_VARS_PER_PDO 64  //ENTRY NUM PER PDO



typedef struct {
  int obj_type;
  char name[MAX_LEN_OF_ECAT_NAME];
  int data_type;
  int bit_offset;
  int bit_length;
  int byte_offset;
  int sub_index; // pdo subinex
  int sdo_index; // sdo index
  int sdo_subindex; // sdo subindex
  char* app_ptr; //pointer to data in app level
  char* sys_ptr; //pointer to data in sys(kernel) level
  char valid; //0: invalid; 1: valid
} ECAT_DATA_VAR_INFO, *PECAT_DATA_VAR_INFO;

typedef struct
{
	/* data area to be set before initialization */
	char name[MAX_LEN_OF_ECAT_NAME];
	unsigned short pdo_index; //from slave point of view: 0x1600 - RxPDO, 0x1a00 - TxPDO
	unsigned short pdo_sub_index;
	/* data area to be filled after initialization */
	int bit_offset;
	int bit_length;	
	char* app_ptr; //pointer to data in app level
	char* sys_ptr; //pointer to data in sys(kernel) level
	int var_count; //entry num
	ECAT_DATA_VAR_INFO vars[MAX_NUM_OF_VARS_PER_PDO];
}ECAT_PDO_OBJECT, *PECAT_PDO_OBJECT;

typedef struct
{
	/* data area to be set before initialization */
	char name[128];	
	int id; //ID of slave (0 means no ID)
	int position; //Position of the slave.
	int vendor; //vendor id
	int product; //product id
	int revision; //revision number
	/* data area to be filled after initialization */
	long long hd_slave; //handle of slave
	int data_obj_input_num; //number of data objects
	ECAT_PDO_OBJECT data_obj_inputs[MAX_NUM_OF_DATA_OBJECTS];
	int data_obj_output_num; //number of data objects
	ECAT_PDO_OBJECT data_obj_outputs[MAX_NUM_OF_DATA_OBJECTS];
}ECATSLAVE,*PECATSLAVE;



typedef struct ECATMASTER
{
	/* data area to be filled after initialization */
	long long hd_master; //handle of master 
	int num_of_slaves;
	ECATSLAVE slaves[MAX_NUM_OF_SLAVES];

	void* app_ptr;
	void* sys_ptr;
	int enabled;
}ECATMASTER, *PECATMASTER;


typedef struct
{
	unsigned int  vid;
	unsigned int  pcode;	
	unsigned int  rno;
	unsigned int  dc;

	char name[EC_MAX_STRING_LENGTH]; /**< Name of the slave. */	
} EC_SLV_CFG_NODE;

/* type of master parameter from config file */
typedef struct EcMstPara
{
	unsigned int nNoOfSla;
	unsigned int nCycleInUs;
	unsigned int nShiftInUs;
	void   *pdo_entry_reg_pool;
	unsigned char    *reg_base;	// for rx pdo register base
	unsigned char    *reg_base_tx;	// for tx pdo register base

} EcMstPara;

typedef struct
{
	unsigned int nNoOfSla;
	unsigned int nCycleInUs;
	unsigned int  nShiftInUs;
	EC_SLV_CFG_NODE nodes[MAX_NUM_OF_SLAVES];
}EcatConfigData;


typedef struct
{
	int data_obj_input_num; //number of data objects
	ECAT_PDO_OBJECT data_obj_inputs[MAX_NUM_OF_DATA_OBJECTS];
	int data_obj_output_num; //number of data objects
	ECAT_PDO_OBJECT data_obj_outputs[MAX_NUM_OF_DATA_OBJECTS];
}SlaveObjects;

/* ecat master app parameters */
typedef struct  _EcatApp
{
	ec_master_t *p_ec_master;  // Point to master
	ec_domain_t *p_ec_domain; // Point to data domain for rx pdo
	ec_domain_t *p_ec_domain_tx; // Point to data domain for tx pdo
	
		
	EcMstPara sMaster;			/* we only support one master at current phase */
	EcatConfigData	scaned_ec_info;

	
	ec_slave_config_t *pConfig[MAX_NUM_OF_SLAVES]; 
	SlaveObjects   slaveObjects[MAX_NUM_OF_SLAVES];

	unsigned int  dc_sync;
}EcatApp;


#pragma pack(pop)



#endif /* _EC_SERVO_H_ */
