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
	rt6590.h

    Abstract:

    Revision History:
    Who          When          What
    ---------    ----------    ----------------------------------------------
 */

#ifndef __MT7601_H__
#define __MT7601_H__

struct _RTMP_ADAPTER;

#define MAC_VERSION		""

#define BBP_VERSION		"MT7601E2_BBP_CSD_20121019"
#define RF_VERSION		"RT6390_RF_Register_20121029"

#define NIC6590_PCIe_DEVICE_ID 0x6590
#define MAX_RF_ID	127
#define MAC_RF_BANK 7

#define MAX_CHECK_COUNT 200
#define MT7601_VALID_EEPROM_VERSION		0x6

#define ENABLE_WLAN_FUN(__WlanFunCtrl)\
{\
	__WlanFunCtrl.field.WLAN_CLK_EN = 1;\
	__WlanFunCtrl.field.WLAN_EN = 1;\
}

#define DISABLE_WLAN_FUN(__WlanFunCtrl)\
{\
	__WlanFunCtrl.field.PCIE_APP0_CLK_REQ = 0;\
	__WlanFunCtrl.field.WLAN_EN = 0;\
	__WlanFunCtrl.field.WLAN_CLK_EN = 0;\
}

#ifdef RTMP_INTERNAL_TX_ALC
#define DEFAULT_BO              4
#define LIN2DB_ERROR_CODE       (-10000)
#define TEMPERATURE_DPD_UNKONW	0xFFFF

#define RF_PA_MODE0_DECODE		0
#define RF_PA_MODE1_DECODE		20480	// 2.5 * 8192
#define RF_PA_MODE2_DECODE		4096	// 0.5 * 8192
#define RF_PA_MODE3_DECODE		4096	// 0.5 * 8192

#define	BW20_MCS_POWER_CCK_1M			((pAd->Tx20MPwrCfgGBand[0] & 0xFF) < 0x20)?(pAd->Tx20MPwrCfgGBand[0] & 0xFF):(CHAR)((pAd->Tx20MPwrCfgGBand[0] & 0xFF) - 0x40)
#define	BW20_MCS_POWER_CCK_2M			((pAd->Tx20MPwrCfgGBand[0] & 0xFF) < 0x20)?(pAd->Tx20MPwrCfgGBand[0] & 0xFF):(CHAR)((pAd->Tx20MPwrCfgGBand[0] & 0xFF) - 0x40)
#define	BW20_MCS_POWER_CCK_5M			(((pAd->Tx20MPwrCfgGBand[0] & 0xFF00) >> 8) < 0x20)?((pAd->Tx20MPwrCfgGBand[0] & 0xFF00) >> 8):(CHAR)(((pAd->Tx20MPwrCfgGBand[0] & 0xFF00) >> 8)-0x40)
#define	BW20_MCS_POWER_CCK_11M			(((pAd->Tx20MPwrCfgGBand[0] & 0xFF00) >> 8) < 0x20)?((pAd->Tx20MPwrCfgGBand[0] & 0xFF00) >> 8):(CHAR)(((pAd->Tx20MPwrCfgGBand[0] & 0xFF00) >> 8)-0x40)
#define	BW40_MCS_POWER_CCK_1M			((pAd->Tx40MPwrCfgGBand[0] & 0xFF) < 0x20)?(pAd->Tx40MPwrCfgGBand[0] & 0xFF) :(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF) < 0x20)-0x40)
#define	BW40_MCS_POWER_CCK_2M			((pAd->Tx40MPwrCfgGBand[0] & 0xFF) < 0x20)?(pAd->Tx40MPwrCfgGBand[0] & 0xFF) :(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF) < 0x20)-0x40)
#define	BW40_MCS_POWER_CCK_5M			(((pAd->Tx40MPwrCfgGBand[0] & 0xFF00) >> 8) < 0x20)?((pAd->Tx40MPwrCfgGBand[0] & 0xFF00) >> 8):(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF00) >> 8)-0x40)
#define	BW40_MCS_POWER_CCK_11M			(((pAd->Tx40MPwrCfgGBand[0] & 0xFF00) >> 8) < 0x20)?((pAd->Tx40MPwrCfgGBand[0] & 0xFF00) >> 8):(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF00) >> 8)-0x40)

#define	BW20_MCS_POWER_OFDM_6M			(((pAd->Tx20MPwrCfgGBand[0] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx20MPwrCfgGBand[0] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx20MPwrCfgGBand[0] & 0xFF0000) >> 16)-0x40)
#define	BW20_MCS_POWER_OFDM_9M			(((pAd->Tx20MPwrCfgGBand[0] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx20MPwrCfgGBand[0] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx20MPwrCfgGBand[0] & 0xFF0000) >> 16)-0x40)
#define	BW20_MCS_POWER_OFDM_12M		(((pAd->Tx20MPwrCfgGBand[0] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx20MPwrCfgGBand[0] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx20MPwrCfgGBand[0] & 0xFF000000) >> 24)-0x40)
#define	BW20_MCS_POWER_OFDM_18M		(((pAd->Tx20MPwrCfgGBand[0] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx20MPwrCfgGBand[0] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx20MPwrCfgGBand[0] & 0xFF000000) >> 24)-0x40)
#define	BW20_MCS_POWER_OFDM_24M		((pAd->Tx20MPwrCfgGBand[1] & 0xFF) < 0x20)?(pAd->Tx20MPwrCfgGBand[1] & 0xFF):(CHAR)((pAd->Tx20MPwrCfgGBand[1] & 0xFF)-0x40)
#define	BW20_MCS_POWER_OFDM_36M		((pAd->Tx20MPwrCfgGBand[1] & 0xFF) < 0x20)?(pAd->Tx20MPwrCfgGBand[1] & 0xFF):(CHAR)((pAd->Tx20MPwrCfgGBand[1] & 0xFF)-0x40)
#define	BW20_MCS_POWER_OFDM_48M		(((pAd->Tx20MPwrCfgGBand[1] & 0xFF00) >> 8) < 0x20)?((pAd->Tx20MPwrCfgGBand[1] & 0xFF00) >> 8):(CHAR)(((pAd->Tx20MPwrCfgGBand[1] & 0xFF00) >> 8)-0x40)
#define	BW20_MCS_POWER_OFDM_54M		(((pAd->Tx20MPwrCfgGBand[1] & 0xFF00) >> 8) < 0x20)?((pAd->Tx20MPwrCfgGBand[1] & 0xFF00) >> 8):(CHAR)(((pAd->Tx20MPwrCfgGBand[1] & 0xFF00) >> 8)-0x40)
#define	BW40_MCS_POWER_OFDM_6M			(((pAd->Tx40MPwrCfgGBand[0] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx40MPwrCfgGBand[0] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF0000) >> 16)-0x40)
#define	BW40_MCS_POWER_OFDM_9M			(((pAd->Tx40MPwrCfgGBand[0] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx40MPwrCfgGBand[0] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF0000) >> 16)-0x40)
#define	BW40_MCS_POWER_OFDM_12M		(((pAd->Tx40MPwrCfgGBand[0] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx40MPwrCfgGBand[0] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF000000) >> 24)-0x40)
#define	BW40_MCS_POWER_OFDM_18M		(((pAd->Tx40MPwrCfgGBand[0] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx40MPwrCfgGBand[0] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx40MPwrCfgGBand[0] & 0xFF000000) >> 24)-0x40)
#define	BW40_MCS_POWER_OFDM_24M		((pAd->Tx40MPwrCfgGBand[1] & 0xFF) < 0x20)?(pAd->Tx40MPwrCfgGBand[1] & 0xFF):(CHAR)((pAd->Tx40MPwrCfgGBand[1] & 0xFF)-0x40)
#define	BW40_MCS_POWER_OFDM_36M		((pAd->Tx40MPwrCfgGBand[1] & 0xFF) < 0x20)?(pAd->Tx40MPwrCfgGBand[1] & 0xFF):(CHAR)((pAd->Tx40MPwrCfgGBand[1] & 0xFF)-0x40)
#define	BW40_MCS_POWER_OFDM_48M		(((pAd->Tx40MPwrCfgGBand[1] & 0xFF00) >> 8) < 0x20)?((pAd->Tx40MPwrCfgGBand[1] & 0xFF00) >> 8):(CHAR)(((pAd->Tx40MPwrCfgGBand[1] & 0xFF00) >> 8)-0x40)
#define	BW40_MCS_POWER_OFDM_54M		(((pAd->Tx40MPwrCfgGBand[1] & 0xFF00) >> 8) < 0x20)?((pAd->Tx40MPwrCfgGBand[1] & 0xFF00) >> 8):(CHAR)(((pAd->Tx40MPwrCfgGBand[1] & 0xFF00) >> 8)-0x40)

#define	BW20_MCS_POWER_HT_MCS0			(((pAd->Tx20MPwrCfgGBand[1] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx20MPwrCfgGBand[1] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx20MPwrCfgGBand[1] & 0xFF0000) >> 16)-0x40)
#define	BW20_MCS_POWER_HT_MCS1			(((pAd->Tx20MPwrCfgGBand[1] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx20MPwrCfgGBand[1] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx20MPwrCfgGBand[1] & 0xFF0000) >> 16)-0x40)
#define	BW20_MCS_POWER_HT_MCS2			(((pAd->Tx20MPwrCfgGBand[1] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx20MPwrCfgGBand[1] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx20MPwrCfgGBand[1] & 0xFF000000) >> 24)-0x40)
#define	BW20_MCS_POWER_HT_MCS3			(((pAd->Tx20MPwrCfgGBand[1] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx20MPwrCfgGBand[1] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx20MPwrCfgGBand[1] & 0xFF000000) >> 24)-0x40)
#define	BW20_MCS_POWER_HT_MCS4			((pAd->Tx20MPwrCfgGBand[2] & 0xFF) < 0x20)?(pAd->Tx20MPwrCfgGBand[2] & 0xFF):(CHAR)((pAd->Tx20MPwrCfgGBand[2] & 0xFF)-0x40)
#define	BW20_MCS_POWER_HT_MCS5			((pAd->Tx20MPwrCfgGBand[2] & 0xFF) < 0x20)?(pAd->Tx20MPwrCfgGBand[2] & 0xFF):(CHAR)((pAd->Tx20MPwrCfgGBand[2] & 0xFF)-0x40)
#define	BW20_MCS_POWER_HT_MCS6			(((pAd->Tx20MPwrCfgGBand[2] & 0xFF00 ) >> 8) < 0x20)?((pAd->Tx20MPwrCfgGBand[2] & 0xFF00 ) >> 8):(CHAR)(((pAd->Tx20MPwrCfgGBand[2] & 0xFF00 ) >> 8)-0x40)
#define	BW20_MCS_POWER_HT_MCS7			(((pAd->Tx20MPwrCfgGBand[2] & 0xFF00 ) >> 8) < 0x20)?((pAd->Tx20MPwrCfgGBand[2] & 0xFF00 ) >> 8):(CHAR)(((pAd->Tx20MPwrCfgGBand[2] & 0xFF00 ) >> 8)-0x40)
#define	BW40_MCS_POWER_HT_MCS0			(((pAd->Tx40MPwrCfgGBand[1] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx40MPwrCfgGBand[1] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx40MPwrCfgGBand[1] & 0xFF0000) >> 16)-0x40)
#define	BW40_MCS_POWER_HT_MCS1			(((pAd->Tx40MPwrCfgGBand[1] & 0xFF0000) >> 16) < 0x20)?((pAd->Tx40MPwrCfgGBand[1] & 0xFF0000) >> 16):(CHAR)(((pAd->Tx40MPwrCfgGBand[1] & 0xFF0000) >> 16)-0x40)
#define	BW40_MCS_POWER_HT_MCS2			(((pAd->Tx40MPwrCfgGBand[1] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx40MPwrCfgGBand[1] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx40MPwrCfgGBand[1] & 0xFF000000) >> 24)-0x40)
#define	BW40_MCS_POWER_HT_MCS3			(((pAd->Tx40MPwrCfgGBand[1] & 0xFF000000) >> 24) < 0x20)?((pAd->Tx40MPwrCfgGBand[1] & 0xFF000000) >> 24):(CHAR)(((pAd->Tx40MPwrCfgGBand[1] & 0xFF000000) >> 24)-0x40)
#define	BW40_MCS_POWER_HT_MCS4			((pAd->Tx40MPwrCfgGBand[2] & 0xFF) < 0x20)?(pAd->Tx40MPwrCfgGBand[2] & 0xFF):(CHAR)((pAd->Tx40MPwrCfgGBand[2] & 0xFF)-0x40)
#define	BW40_MCS_POWER_HT_MCS5			((pAd->Tx40MPwrCfgGBand[2] & 0xFF) < 0x20)?(pAd->Tx40MPwrCfgGBand[2] & 0xFF):(CHAR)((pAd->Tx40MPwrCfgGBand[2] & 0xFF)-0x40)
#define	BW40_MCS_POWER_HT_MCS6			(((pAd->Tx40MPwrCfgGBand[2] & 0xFF00) >> 8) < 0x20)?(pAd->Tx40MPwrCfgGBand[2] & 0xFF00):(CHAR)((pAd->Tx40MPwrCfgGBand[2] & 0xFF00)-0x40)
#define	BW40_MCS_POWER_HT_MCS7			(((pAd->Tx40MPwrCfgGBand[2] & 0xFF00) >> 8) < 0x20)?(pAd->Tx40MPwrCfgGBand[2] & 0xFF00):(CHAR)((pAd->Tx40MPwrCfgGBand[2] & 0xFF00)-0x40)

#define	BB_PA_MODE_CCK_1M				(pAd->chipCap.TxALCData.PAModeCCK[0])
#define	BB_PA_MODE_CCK_2M				(pAd->chipCap.TxALCData.PAModeCCK[1]) 
#define	BB_PA_MODE_CCK_5M				(pAd->chipCap.TxALCData.PAModeCCK[2]) 
#define	BB_PA_MODE_CCK_11M				(pAd->chipCap.TxALCData.PAModeCCK[3]) 

#define	BB_PA_MODE_OFDM_6M				(pAd->chipCap.TxALCData.PAModeOFDM[0])
#define	BB_PA_MODE_OFDM_9M				(pAd->chipCap.TxALCData.PAModeOFDM[1])
#define	BB_PA_MODE_OFDM_12M				(pAd->chipCap.TxALCData.PAModeOFDM[2]) 
#define	BB_PA_MODE_OFDM_18M				(pAd->chipCap.TxALCData.PAModeOFDM[3]) 
#define	BB_PA_MODE_OFDM_24M				(pAd->chipCap.TxALCData.PAModeOFDM[4])
#define	BB_PA_MODE_OFDM_36M				(pAd->chipCap.TxALCData.PAModeOFDM[5])
#define	BB_PA_MODE_OFDM_48M				(pAd->chipCap.TxALCData.PAModeOFDM[6])
#define	BB_PA_MODE_OFDM_54M				(pAd->chipCap.TxALCData.PAModeOFDM[7])

#define	BB_PA_MODE_HT_MCS0				(pAd->chipCap.TxALCData.PAModeHT[0])
#define	BB_PA_MODE_HT_MCS1				(pAd->chipCap.TxALCData.PAModeHT[1]) 
#define	BB_PA_MODE_HT_MCS2				(pAd->chipCap.TxALCData.PAModeHT[2]) 
#define	BB_PA_MODE_HT_MCS3				(pAd->chipCap.TxALCData.PAModeHT[3]) 
#define	BB_PA_MODE_HT_MCS4				(pAd->chipCap.TxALCData.PAModeHT[4])
#define	BB_PA_MODE_HT_MCS5				(pAd->chipCap.TxALCData.PAModeHT[5])
#define	BB_PA_MODE_HT_MCS6				(pAd->chipCap.TxALCData.PAModeHT[6])
#define	BB_PA_MODE_HT_MCS7				(pAd->chipCap.TxALCData.PAModeHT[7])
#define	BB_PA_MODE_HT_MCS8				(pAd->chipCap.TxALCData.PAModeHT[8])
#define	BB_PA_MODE_HT_MCS9				(pAd->chipCap.TxALCData.PAModeHT[9]) 
#define	BB_PA_MODE_HT_MCS10				(pAd->chipCap.TxALCData.PAModeHT[10]) 
#define	BB_PA_MODE_HT_MCS11				(pAd->chipCap.TxALCData.PAModeHT[11]) 
#define	BB_PA_MODE_HT_MCS12				(pAd->chipCap.TxALCData.PAModeHT[12])
#define	BB_PA_MODE_HT_MCS13				(pAd->chipCap.TxALCData.PAModeHT[13])
#define	BB_PA_MODE_HT_MCS14				(pAd->chipCap.TxALCData.PAModeHT[14])
#define	BB_PA_MODE_HT_MCS15				(pAd->chipCap.TxALCData.PAModeHT[15])

VOID MT7601_TssiDcGainCalibration(struct _RTMP_ADAPTER *pAd);

enum TEMPERATURE_MODE {
	TEMPERATURE_MODE_NORMAL,
	TEMPERATURE_MODE_LOW,
	TEMPERATURE_MODE_HIGH,
};

typedef struct _MT7601_TX_ALC_DATA {
	INT32	PowerDiffPre;
	INT32	TemperatureDPD;
	INT32	MT7601_TSSI_T0_Delta_Offset;
	INT32	CurrentTemperature;
	INT16	TSSI_DBOFFSET_HVGA;
	INT16	TSSI0_DB;
	UCHAR	TssiSlope;
	CHAR	TssiDC0;
	CHAR	TssiDC0_HVGA;
	CHAR	TemperatureRef25C;
	UCHAR	TemperatureMode;
	BOOLEAN	TSSI_USE_HVGA;
	BOOLEAN TssiTriggered;
	CHAR	MT7601_TSSI_OFFSET[3];
	UINT16	PAModeCCK[4];
	UINT16	PAModeOFDM[8];
	UINT16	PAModeHT[16];
} MT7601_TX_ALC_DATA, *PMT7601_TX_ALC_DATA;

#endif /* RTMP_INTERNAL_TX_ALC */

/*
	rsv: Reserved
	tcp: packet type, tcp : 1, udp:0
	tups: TCP/UDP header start offset (in unit of DWORD)
	ips: IP start offset (in unit of byte), base address of ips is the beginning of TXWI
	mss: Max Segment size (in unit of byte)
*/
#ifdef RT_BIG_ENDIAN
typedef struct _TSO_INFO_{
	UINT32 mss:16;
	UINT32 ips:8;
	UINT32 tups:6;	
	UINT32 tcp:1;	
	UINT32 rsv:1;
}TSO_INFO;
#else
typedef struct _TSO_INFO_{
	UINT32 rsv:1;
	UINT32 tcp:1;
	UINT32 tups:6;
	UINT32 ips:8;
	UINT32 mss:16;
}TSO_INFO;
#endif /* RT_BIG_ENDIAN */

/* 
 * Frequency plan item  for MT7601
 * K_R17[7:0]: sdm_k[7:0]
 * K_R18[7:0]: sdm_k[15:8]
 * K_R19[1:0]: sdm_k[17:16]
 * K_R19[3]: sdm_clk_sel
 * R_R20[7:0]: sdm_n[7:0]
 */
typedef struct _MT7601_FREQ_ITEM {
	UINT8 Channel;
	UINT8 K_R17;
	UINT8 K_R18;
	UINT8 K_R19;
	UINT8 N_R20;
} MT7601_FREQ_ITEM;

#define RF_G_BAND 		0x01
#define RF_A_BAND 		0x02
#define RF_A_BAND_LB	0x04
#define RF_A_BAND_MB	0x08
#define RF_A_BAND_HB	0x10
typedef struct _RT6590_RF_SWITCH_ITEM {
	UCHAR Bank;
	UCHAR Register;
	UCHAR Band; /* G_Band, A_Band_LB, A_Band_MB, A_Band_HB */
	UCHAR BW;
	UCHAR Value;
} RT6590_RF_SWITCH_ITEM, *PRT6590_RF_SWITCH_ITEM;

VOID MT7601_Init(struct _RTMP_ADAPTER *pAd);
VOID MT7601_RXDC_CAL(struct _RTMP_ADAPTER *pAd);
INT MT7601_ReadChannelPwr(struct _RTMP_ADAPTER *pAd);
VOID MT7601_INIT_CAL(struct _RTMP_ADAPTER *pAd);
VOID MT7601DisableTxRx(struct _RTMP_ADAPTER *pAd, UCHAR Level);
VOID dump_bw_info(struct _RTMP_ADAPTER *pAd);
#ifdef RTMP_INTERNAL_TX_ALC
VOID MT7601_TemperatureCompensation(
	IN struct _RTMP_ADAPTER *pAd,
	IN CHAR Temperature);
INT16 lin2dBd(UINT16 linearValue);
VOID MT7601_EnableTSSI(struct _RTMP_ADAPTER *pAd);
#endif /* RTMP_INTERNAL_TX_ALC */

#endif /* __MT7601_H__ */

