/****************************************************************************
 * Ralink Tech Inc.
 * 4F, No. 2 Technology 5th Rd.
 * Science-based Industrial Park
 * Hsin-chu, Taiwan, R.O.C.
 * (c) Copyright 2002, Ralink Technology, Inc.
 *
 * All rights reserved. Ralink's source code is an unpublished work and the
 * use of a copyright notice does not imply otherwise. This source code
 * contains confidential trade secret material of Ralink Tech. Any attemp
 * or participation in deciphering, decoding, reverse engineering or in any
 * way altering the source code is stricitly prohibited, unless the prior
 * written consent of Ralink Technology, Inc. is obtained.
 ****************************************************************************

    Module Name:
	ral_omac_usb.h
 
    Abstract:
 
    Revision History:
    Who          When          What
    ---------    ----------    ----------------------------------------------
 */

#ifndef __RAL_OMAC_USB_H__
#define __RAL_OMAC_USB_H__
#define CMB_CTRL		0x20
#ifdef RT_BIG_ENDIAN
typedef union _CMB_CTRL_STRUC{
	struct{
		UINT32       	LDO0_EN:1;
		UINT32       	LDO3_EN:1;
		UINT32       	LDO_BGSEL:2;
		UINT32       	LDO_CORE_LEVEL:4;
		UINT32       	PLL_LD:1;
		UINT32       	XTAL_RDY:1;
		UINT32       	Rsv:2;
		UINT32		LDO25_FRC_ON:1;//4      
		UINT32		LDO25_LARGEA:1;
		UINT32		LDO25_LEVEL:2;
		UINT32		AUX_OPT_Bit15_Two_AntennaMode:1;
		UINT32		AUX_OPT_Bit14_TRSW1_as_GPIO:1;
		UINT32		AUX_OPT_Bit13_GPIO7_as_GPIO:1;
		UINT32		AUX_OPT_Bit12_TRSW0_as_WLAN_ANT_SEL:1;
		UINT32		AUX_OPT_Bit11_Rsv:1;
		UINT32		AUX_OPT_Bit10_NotSwap_WL_LED_ACT_RDY:1;
		UINT32		AUX_OPT_Bit9_GPIO3_as_GPIO:1;
		UINT32		AUX_OPT_Bit8_AuxPower_Exists:1;
		UINT32		AUX_OPT_Bit7_KeepInterfaceClk:1;
		UINT32		AUX_OPT_Bit6_KeepXtal_On:1;
		UINT32		AUX_OPT_Bit5_RemovePCIePhyClk_BTOff:1;
		UINT32		AUX_OPT_Bit4_RemovePCIePhyClk_WLANOff:1;
		UINT32		AUX_OPT_Bit3_PLLOn_L1:1;
		UINT32		AUX_OPT_Bit2_PCIeCoreClkOn_L1:1;
		UINT32		AUX_OPT_Bit1_PCIePhyClkOn_L1:1;	
		UINT32		AUX_OPT_Bit0_InterfaceClk_40Mhz:1;
	}field;
	UINT32 word;
}CMB_CTRL_STRUC, *PCMB_CTRL_STRUC;
#else
typedef struct field14{
		UINT32		AUX_OPT_Bit0_InterfaceClk_40Mhz:1;
		UINT32		AUX_OPT_Bit1_PCIePhyClkOn_L1:1;	
		UINT32		AUX_OPT_Bit2_PCIeCoreClkOn_L1:1;
		UINT32		AUX_OPT_Bit3_PLLOn_L1:1;
		UINT32		AUX_OPT_Bit4_RemovePCIePhyClk_WLANOff:1;
		UINT32		AUX_OPT_Bit5_RemovePCIePhyClk_BTOff:1;
		UINT32		AUX_OPT_Bit6_KeepXtal_On:1;
		UINT32		AUX_OPT_Bit7_KeepInterfaceClk:1;
		UINT32		AUX_OPT_Bit8_AuxPower_Exists:1;
		UINT32		AUX_OPT_Bit9_GPIO3_as_GPIO:1;
		UINT32		AUX_OPT_Bit10_NotSwap_WL_LED_ACT_RDY:1;	
		UINT32		AUX_OPT_Bit11_Rsv:1;
		UINT32		AUX_OPT_Bit12_TRSW0_as_WLAN_ANT_SEL:1;
		UINT32		AUX_OPT_Bit13_GPIO7_as_GPIO:1;
		UINT32		AUX_OPT_Bit14_TRSW1_as_GPIO:1;
		UINT32		AUX_OPT_Bit15_Two_AntennaMode:1;
		UINT32		LDO25_LEVEL:2;
		UINT32		LDO25_LARGEA:1;
		UINT32		LDO25_FRC_ON:1;//4      
		UINT32       	Rsv:2;
		UINT32       	XTAL_RDY:1;
		UINT32       	PLL_LD:1;
		UINT32       	LDO_CORE_LEVEL:4;
		UINT32       	LDO_BGSEL:2;
		UINT32       	LDO3_EN:1;
		UINT32       	LDO0_EN:1;
	}field14;

typedef union _CMB_CTRL_STRUC{
	field14 field;
	UINT32 word;
}CMB_CTRL_STRUC, *PCMB_CTRL_STRUC;
#endif


#endif /*__RAL_OMAC_USB_H__ */

