 /**
	\file
	EtherCAT CANopen over EtherCAT data sturcture.
 */
 
/*****************************************************************************/

#ifndef __ECAT_COE_H__
#define __ECAT_COE_H__

#include <sys/types.h>
#include <stdint.h>

/*****************************************************************************/
/**
   Command definition
*/
#define NOP						0x00
#define SHUT_DOWN				0x06
#define SWITCH_ON				0x07
#define	SV_ON					0x0F
#define	SV_OFF					0x06
#define	FALT_RST				0x80
#define	HM_START				0x1F
#define HM_FINISH     			0xEF
#define	SV_ON_SYNC				0x1F
/*****************************************************************************/
/**
   controlword bit definition.
*/
typedef struct _ctrwrd_bits_t
{
	uint8_t	so	:		1;		//BIT0: switch on
	uint8_t	ev	:		1;		//BIT1: enable voltage
	uint8_t	qs	:		1;		//BIT2: quick stop
	uint8_t	eo	:		1;		//BIT3: enable operation
	uint8_t	oms1:		1;		//BIT4: oms1 pp-New set-point; hm-Homing start; 
										//		  ip-Enable interpolation;
	uint8_t	oms2:		1;		//BIT5: oms2 pp-Change set immediately;
	uint8_t	oms3:		1;		//BIT6: oms3 pp-abs 0/rel 1;
	uint8_t	fr	:		1;		//BIT7: fault reset
	uint8_t	h	:		1;		//BIT8: hault
	uint8_t	oms4:		1;		//BIT9: oms4 change on set-point;
	uint8_t	r	:		1;		//BIT10: reserved
	uint8_t	ms1	:		1;		//BIT11: manufacturer-specific1
	uint8_t	ms2	:		1;		//BIT12: manufacturer-specific2
	uint8_t	ms3	:		1;		//BIT13: manufacturer-specific3
	uint8_t	ms4	:		1;		//BIT14: manufacturer-specific4
	uint8_t	ms5	:		1;		//BIT15: manufacturer-specific5	
}ctrwrd_bits_t;
/*****************************************************************************/
/**
   controlword definition.
*/
typedef union _ctrwrd_t
{
	uint16_t			all;
	ctrwrd_bits_t		bit;
}ctrwrd_t;
/*****************************************************************************/
/**
   statusword bit definition.
*/
typedef struct _stswrd_bits_t
{
	uint8_t	rtso:		1;		//BIT0: ready to switch on 
	uint8_t	so	:		1;		//BIT1: switched on
	uint8_t	oe	:		1;		//BIT2: operation enabled
	uint8_t	f	:		1;		//BIT3: fault
	uint8_t	ve	:		1;		//BIT4:  voltage enabled
	uint8_t	qs	:		1;		//BIT5: quick stop
	uint8_t	sod	:		1;		//BIT6: switch on disabled
	uint8_t	w	:		1;		//BIT7: warning
	uint8_t	ms1	:		1;		//BIT8: manufacturer-specific1
	uint8_t	rm	:		1;		//BIT9: remote
	uint8_t	tr	:		1;		//BIT10: target reached
	uint8_t	ila	:		1;		//BIT11:  internal limit active
	uint8_t	oms1:		1;		//BIT12: oms1 pp-Set-point acknowledge; hm-Homing attained;
										//		ip-ip  mode active;csp-Target position ignored; 
										//		csv-Target velocity ignored;cst-Target torque ignored; 
	uint8_t	oms2:		1;		//BIT13: oms1 pp-Following error; hm-Homing error; csp-Following error; 
	uint8_t	ms2	:		1;		//BIT14: manufacturer-specific2
	uint8_t	ms3	:		1;		//BIT15: manufacturer-specific3	
}stswrd_bits_t;
/*****************************************************************************/
/**
   statusword definition.
*/
typedef union	_stswrd_t
{
	uint16_t			all;
	stswrd_bits_t		bit;
}stswrd_t;
/*****************************************************************************/
#endif

