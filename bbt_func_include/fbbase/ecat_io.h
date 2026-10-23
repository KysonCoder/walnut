/** \file ecat_io.h
 * \brief Ethercat IO access
 *
 * Usage : Use function to access IO: DI/DO/AI/AO
 *
 *
 *
 * 
 *
 * (c) AT  2022.02.02
 */
#ifndef __ECAT_IO_H_
	#define __ECAT_IO_H_

#include "ecat_type_def.h"


/*remote IO parameters */

#pragma pack(push, 4)

#define MAX_NUM_OF_DI_PER_NODE 256
#define MAX_NUM_OF_DO_PER_NODE 256
#define MAX_NUM_OF_AI_PER_NODE 256
#define MAX_NUM_OF_AO_PER_NODE 256
#define MAX_NUM_OF_IO_NODES 64

/*------------------------- Private Typedef -----------------------*/
typedef enum
{
	io_cmd_goto_standby = 0,
	io_cmd_goto_run	
}ECAT_IO_CMD;

typedef enum
{
	io_status_unknown,
	io_status_error,
	io_status_inited,
	io_status_enabled
}IO_STATUS;

typedef enum
{
	io_type_di,
	io_type_do,
	io_type_ai,
	io_type_ao
}IO_TYPE;

typedef struct
{
	int vendor;
	int product;

	//Cyclic data, process data object
	//inputs
	unsigned int pdo_index_ctrl_word; //16bit
	unsigned int pdo_subindex_ctrl_word;
	unsigned int pdo_index_down_request; //request acknowledge from main computer to io board, 16bit
	unsigned int pdo_subindex_down_request;	

	//outputs
	unsigned int pdo_index_status_word; //16bit
	unsigned int pdo_subindex_status_word;
	unsigned int pdo_index_ioinputs;
	unsigned int pdo_subindex_ioinuts; //32bit
	unsigned int pdo_index_up_request;	//request from io board to main computer, 16bit
	unsigned int pdo_subindex_up_request;	//request from io board to main computer, 16bit
	unsigned int pdo_index_error;	//request from io board to main computer, 16bit
	unsigned int pdo_subindex_error;	//request from io board to main computer, 16bit
}ECAT_IO_NODE_CONFIG, *PECAT_IO_NODE_CONFIG;

/* pdo data in servo node */
typedef struct
{
   ECAT_DATA_VAR_INFO di[MAX_NUM_OF_DI_PER_NODE];
   ECAT_DATA_VAR_INFO doo[MAX_NUM_OF_DO_PER_NODE]; //do is a key word of C
   ECAT_DATA_VAR_INFO ai[MAX_NUM_OF_AI_PER_NODE];
   ECAT_DATA_VAR_INFO ao[MAX_NUM_OF_AO_PER_NODE];
   int valid_flag;
   int val_di[MAX_NUM_OF_DI_PER_NODE];
   int val_doo[MAX_NUM_OF_DO_PER_NODE];
   int val_ai[MAX_NUM_OF_AI_PER_NODE];
   int val_ao[MAX_NUM_OF_AO_PER_NODE];
}ECAT_IO_NODE_PDO_DATA, *PECAT_IO_NODE_PDO_DATA;

typedef struct 
{
	long long hd_node;
	int index;
	ECAT_IO_NODE_PDO_DATA node_data;
	ECAT_IO_NODE_CONFIG node_config;
	int di_num;
	int do_num;
	int ai_num;
	int ao_num;
}ECAT_IO_NODE, *PECAT_IO_NODE;


typedef struct
{
	long long hd_master;
	long long hd_dataset;
	int num_of_io_nodes;
	ECAT_IO_NODE nodes[MAX_NUM_OF_IO_NODES];
	IO_STATUS io_status;
	void* app_ptr;
	void* sys_ptr;
	char valid;
}ECAT_IO_SYSTEM, *PECAT_IO_SYSTEM;

 int ecat_io_init(PECAT_IO_SYSTEM pIOSys, PECATMASTER pMaster);
 int  ecat_io_data_exchange(void);
 int ecat_io_get_io_num(PECAT_IO_SYSTEM pIOSys, int* di_num, int *do_num, int* ai_num, int* ao_num);

 int ecat_io_get_di(int index, unsigned char* val);
 int ecat_io_set_do(int index, int val);
 int ecat_io_get_di(int board, int channel, unsigned char* val);
 int ecat_io_set_do(int board, int channel, int val);
 int ecat_io_get_ai(int index, int* val);
 int ecat_io_set_ao(int index, int val);
 int ecat_io_set_ao(int board, int channel, int val);
 int ecat_io_get_ai(int board, int channel, int* val);
  
#pragma pack(pop)

#endif //__ECAT_IO_H_
