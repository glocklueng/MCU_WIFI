/*
 * WIFI���õĸ��๦����ο�net/rt5572.../module/common/ap_cfg.c�ļ�
 *
 */


#define DEBUG
#include "wifi.h"
#include "drivers.h"
#include "tcpapp.h"

PRTMP_ADAPTER p_ad = 0;

/*
 * @brief  wifi������ɵĻص�����
 *
 */
void link_finish(void *ptmr, void *parg)
{
	p_dbg_enter;
}

/**
 * @brief  ����wifiģʽ
 * @param  type : INT_MAIN, INT_APCLI
 * @retval ok: 0, err: -1
 */
int wifi_set_mode(int type)
{
	int set_type;
	if (type == INT_MAIN)
	{

		Set_ApCli_Enable_Proc(p_ad, "0");
		set_if_type(p_ad, INT_MAIN);
	}
	else if (type == INT_APCLI)
	{
		set_if_type(p_ad, INT_APCLI);
	}
	else
		return  - 1;

	lwip_netif_init(); //�л�ģʽ����Ҫ���³�ʼ��lwip�����豸����Ҫ��mac��ַ�ĸı�
}



/**
 * @brief  ����wifiƵ��
 * @param  channel : 1 - 14
 * @retval ok: 0, err: -1
 */
int wifi_set_channel(int channel)
{
	char tmp[8];
	sprintf(tmp, "%d", channel);
	Set_Channel_Proc(p_ad, tmp);

	return 0;
}



/**
 * @brief  ����staģʽ�µ�����
 * @param  type : ȡֵ����
 * 						"WEP"  WEP����
 * 						"WPA"	WPA��WPA2����
 *				 key: ��������	
 * @retval ok: 0, err: -1
 */
int wifi_client_add_key(char *type, char *key)
{
	if (strcmp(type, "WEP") == 0)
	{
		Set_ApCli_Key1_Proc(p_ad, key);
		Set_ApCli_DefaultKeyID_Proc(p_ad, "1"); //Ϊ�˽�ʡ�ڴ棬����ֻ��һ��wepkey

	}
	else if (strcmp(type, "WPA") == 0)
	{
		Set_ApCli_WPAPSK_Proc(p_ad, key);
	}
	else
		return  - 1;

	return 0;
}



/**
 * @brief  ����wifiģʽΪAP����
 *
 * @param  type : name:�������ƣ�key:����
key_type
ȡֵ:KEY_NONE = 0
KEY_WEP
KEY_WPA
KEY_WPA2
 * @retval ok: 0, err: -1
 */
int wifi_ap_cfg(char *name, char *key, int key_type, int channel, int hidden)
{
	char tmp_str[8];
	wifi_set_mode(INT_MAIN);


	sprintf(tmp_str, "%d", hidden);
	Set_HideSSID_Proc(p_ad, tmp_str);

	sprintf(tmp_str, "%d", channel);
	if (key_type == KEY_NONE)
	{
		Set_Channel_Proc(p_ad, tmp_str);

		AsicRemoveSharedKeyEntry(p_ad, 0, 0);

		Set_AP_AuthMode_Proc(p_ad, "OPEN");
		Set_AP_EncrypType_Proc(p_ad, "NONE");

		Set_AP_SSID_Proc(p_ad, name);
	}
	else if (key_type == KEY_WEP)
	{
		Set_Channel_Proc(p_ad, tmp_str);

		Set_AP_DefaultKeyID_Proc(p_ad, "1");
		Set_AP_Key1_Proc(p_ad, key);

		Set_AP_AuthMode_Proc(p_ad, "SHARED");
		Set_AP_EncrypType_Proc(p_ad, "WEP");

		Set_AP_SSID_Proc(p_ad, name);
	}
	else if (key_type == KEY_WPA)
	{
		Set_AP_AuthMode_Proc(p_ad, "WPAPSK");
		Set_AP_EncrypType_Proc(p_ad, "TKIP");

		Set_AP_SSID_Proc(p_ad, name); //Set_AP_WPAPSK_ProcҪ�õ�ssid,���Ա����ȵ���

		Set_AP_WPAPSK_Proc(p_ad, key);

		Set_Channel_Proc(p_ad, tmp_str);
	}
	else if (key_type == KEY_WPA2)
	{
		Set_AP_AuthMode_Proc(p_ad, "WPA2PSK");
		Set_AP_EncrypType_Proc(p_ad, "AES");

		Set_AP_SSID_Proc(p_ad, name);

		Set_AP_WPAPSK_Proc(p_ad, key);

		Set_Channel_Proc(p_ad, tmp_str);
	}



	return 0;
}



/**
 * @brief  ɨ�踽����AP
 * @param
 * @retval ok: 0, err: -1
 */
BSS_ENTRY_SIMPLE *wifi_scan()
{
	Set_SiteSurvey_Proc(p_ad, ""); //�ڶ�����������ָ��Ҫ������essid�����ñ�ʾ����ȫ��
	//��ʱ3s�ȴ�ɨ�����,Ҳ���Եȴ��¼�֪ͨ��ɨ������¼�����0x0211
	sleep(3000);

	return BssTabCloneSimple(&p_ad->ScanTabSimple); //����һ��ɨ�赽�Ľ��
}


/**
 * @brief  wifi �Ƿ��Ѿ�����
 *
 */
int is_wifi_connected()
{
	int ret = 0;
	int if_type;

	if_type = get_if_type(p_ad);

	if (if_type == INT_APCLI)
	{
		if (p_ad->ApCfg.ApCliTab[0].CtrlCurrState == APCLI_CTRL_CONNECTED)
			ret = 1;
	}
	else if (if_type == INT_MAIN)
	{
		ret = p_ad->ApCfg.MBSSID[0].StaCount;
	}
	else
		p_err("unkown if_type: %d", if_type);

	return ret;
}

/**
 * @brief  wifi����
 * @param
 *							
 *				 essid: ��AP����
 *				 key: ��������	
 * @retval ok: 0, err: -1
 */
int wifi_connect(char *authmode, char *encryp, char *essid, char *key, char *channel)
{
	wifi_set_mode(INT_APCLI);

	Set_Channel_Proc(p_ad, channel);
	Set_ApCli_Enable_Proc(p_ad, "1");
	Set_ApCli_AuthMode_Proc(p_ad, authmode);
	Set_ApCli_EncrypType_Proc(p_ad, encryp);
	Set_ApCli_Ssid_Proc(p_ad, essid);

	if ((strcmp(authmode, "WPAPSK") == 0) || (strcmp(authmode, "WPA2PSK") == 0))
	{
		wifi_client_add_key("WPA", key);
	}
	else
	{
		wifi_client_add_key("WEP", key);
	}

	Set_ApCli_Enable_Proc(p_ad, "1");
#ifdef MT7601
	Set_SiteSurvey_Proc(p_ad, essid);
	sleep(3000);
#endif
	ApCliIfUp(p_ad);

	return 0;
}

/**
 * @brief  �Ͽ�����
 */
int wifi_disconnect()
{
	int if_type;

	if_type = get_if_type(p_ad);

	if (if_type == INT_APCLI)
	{
		Set_ApCli_Enable_Proc(p_ad, "0");
		set_if_type(p_ad, INT_MAIN);
	}
	else if (if_type == INT_MAIN)
	{
		Set_DisConnectAllSta_Proc(p_ad, "");
	}

	return 0;
}


/**
 * @brief  ��ȡmac��ַ��mac��ַ����NICReadEEPROMParameters�����������õģ�
 * �û��������������Ϊ�����ַ
 * @param  mac_addr : mac��ַ 6BYTE
 */
int wifi_get_mac(unsigned char *mac_addr)
{
	memcpy(mac_addr, p_ad->CurrentAddress, MAC_ADDR_LEN);
	return 0;
}


/**
 * @brief  ��ȡWIFI���ӵ�ͳ����Ϣ(�ź�ǿ��...)
 *
 * @param  pStats : ָ��iw_statistics��ָ��
 */
extern struct netif *p_netif;

int wifi_get_stats(struct iw_statistics *pStats)
{
	struct iw_statistics *stat;
	int if_type;

	if_type = get_if_type(p_ad);

	if (if_type == INT_APCLI)
		stat = rt28xx_get_wireless_stats(p_ad->ApCfg.ApCliTab[0].dev);
	else if (if_type == INT_MAIN)
		stat = rt28xx_get_wireless_stats(p_ad->net_dev);

	if (stat)
		memcpy(pStats, stat, sizeof(struct iw_statistics));


	return 0;
}


/**
 * @brief  WIFI �¼��ص�����,����ɨ����ɣ����ӶϿ����¼�֪ͨ
 * @param  type :ȡֵ����
#define	IW_ASSOC_EVENT_FLAG                         			0x0200
#define	IW_DISASSOC_EVENT_FLAG                      		0x0201
#define	IW_DEAUTH_EVENT_FLAG                      			0x0202
#define	IW_AGEOUT_EVENT_FLAG                      			0x0203
#define	IW_COUNTER_MEASURES_EVENT_FLAG              	0x0204
#define	IW_REPLAY_COUNTER_DIFF_EVENT_FLAG          	0x0205
#define	IW_RSNIE_DIFF_EVENT_FLAG           				0x0206
#define	IW_MIC_DIFF_EVENT_FLAG           				0x0207
#define IW_ICV_ERROR_EVENT_FLAG						0x0208
#define IW_MIC_ERROR_EVENT_FLAG						0x0209
#define IW_GROUP_HS_TIMEOUT_EVENT_FLAG				0x020A
#define	IW_PAIRWISE_HS_TIMEOUT_EVENT_FLAG			0x020B
#define IW_RSNIE_SANITY_FAIL_EVENT_FLAG				0x020C
#define IW_SET_KEY_DONE_WPA1_EVENT_FLAG			0x020D
#define IW_SET_KEY_DONE_WPA2_EVENT_FLAG			0x020E
#define IW_STA_LINKUP_EVENT_FLAG						0x020F
#define IW_STA_LINKDOWN_EVENT_FLAG					0x0210
#define IW_SCAN_COMPLETED_EVENT_FLAG				0x0211
#define IW_SCAN_ENQUEUE_FAIL_EVENT_FLAG				0x0212
#define IW_CHANNEL_CHANGE_EVENT_FLAG				0x0213
#define IW_STA_MODE_EVENT_FLAG						0x0214
#define IW_MAC_FILTER_LIST_EVENT_FLAG				0x0215
#define IW_AUTH_REJECT_CHALLENGE_FAILURE			0x0216
#define IW_SCANNING_EVENT_FLAG						0x0217
#define IW_START_IBSS_FLAG							0x0218
#define IW_JOIN_IBSS_FLAG							0x0219
#define IW_SHARED_WEP_FAIL							0x021A
#define IW_WPS_END_EVENT_FLAG						0x021B
 */
typedef struct _WIRELESS_EVENT_STR
{
	uint32_t flag;
	char *str;
} WIRELESS_EVENT_STR;
const WIRELESS_EVENT_STR wireless_event_str[] =
{
	{(uint32_t)IW_ASSOC_EVENT_FLAG, "had associated successfully"	},
	{(uint32_t)IW_DISASSOC_EVENT_FLAG, "had disassociated"	},
	{(uint32_t)IW_DEAUTH_EVENT_FLAG, "had deauthenticated"},
	{(uint32_t)IW_AGEOUT_EVENT_FLAG, "had been aged-out and disassociated"},
	{(uint32_t)IW_COUNTER_MEASURES_EVENT_FLAG, "occurred CounterMeasures attack"},
	{(uint32_t)IW_REPLAY_COUNTER_DIFF_EVENT_FLAG, "occurred replay counter different in Key Handshaking"},
	{(uint32_t)IW_RSNIE_DIFF_EVENT_FLAG, "occurred RSNIE different in Key Handshaking"},
	{(uint32_t)IW_MIC_DIFF_EVENT_FLAG, "occurred MIC different in Key Handshaking"},
	{(uint32_t)IW_ICV_ERROR_EVENT_FLAG, "occurred ICV error in RX"},
	{(uint32_t)IW_MIC_ERROR_EVENT_FLAG, "occurred MIC error in RX"},
	{(uint32_t)IW_GROUP_HS_TIMEOUT_EVENT_FLAG, "Group Key Handshaking timeout"},
	{(uint32_t)IW_PAIRWISE_HS_TIMEOUT_EVENT_FLAG, "Pairwise Key Handshaking timeout"},
	{(uint32_t)IW_RSNIE_SANITY_FAIL_EVENT_FLAG, "RSN IE sanity check failure"},
	{(uint32_t)IW_SET_KEY_DONE_WPA1_EVENT_FLAG, "set key done in WPA/WPAPSK"},
	{(uint32_t)IW_SET_KEY_DONE_WPA2_EVENT_FLAG, "set key done in WPA2/WPA2PSK"}	,
	{(uint32_t)IW_STA_LINKUP_EVENT_FLAG, "connects with our wireless client"},
	{(uint32_t)IW_STA_LINKDOWN_EVENT_FLAG, "disconnects with our wireless client"},
	{(uint32_t)IW_SCAN_COMPLETED_EVENT_FLAG, "scan completed"},
	{(uint32_t)IW_SCAN_ENQUEUE_FAIL_EVENT_FLAG, "scan terminate!! Busy!! Enqueue fail!!"},
	{(uint32_t)IW_CHANNEL_CHANGE_EVENT_FLAG, "channel switch to "},
	{(uint32_t)IW_STA_MODE_EVENT_FLAG, "wireless mode is not support"},
	{(uint32_t)IW_MAC_FILTER_LIST_EVENT_FLAG, "blacklisted in MAC filter list"},
	{(uint32_t)IW_AUTH_REJECT_CHALLENGE_FAILURE, "Authentication rejected because of challenge failure"},
	{(uint32_t)IW_SCANNING_EVENT_FLAG, "Scanning"},
	{(uint32_t)IW_START_IBSS_FLAG, "Start a new IBSS"},
	{(uint32_t)IW_JOIN_IBSS_FLAG, "Join the IBSS"},
	{(uint32_t)IW_SHARED_WEP_FAIL, "Shared WEP fail"},
	{(uint32_t)IW_WPS_END_EVENT_FLAG, "WPS_END_EVENT"},

};

int event_callback(int type)
{
	if ((type - IW_SYS_EVENT_FLAG_START) < sizeof(wireless_event_str))
		p_dbg("[wireless event:%s] ", wireless_event_str[type - IW_SYS_EVENT_FLAG_START].str);
	else
		p_dbg("[unkown event:0x%x] ", type);
	return 0;
}

extern uint8_t g_mac_addr[6];

#define SHA1_MAC_LEN 20
//sha1ɢ���㷨
int pbkdf2_sha1(const char *passphrase, const char *ssid, size_t ssid_len, int iterations, u8 *buf, size_t buflen)
{
	unsigned int count = 0;
	unsigned char *pos = buf;
	size_t left = buflen, plen;
	unsigned char digest[SHA1_MAC_LEN];

	while (left > 0)
	{
		count++;
		if (pbkdf2_sha1_f(passphrase, ssid, ssid_len, iterations, count, digest))
			return  - 1;
		plen = left > SHA1_MAC_LEN ? SHA1_MAC_LEN : left;
		memcpy(pos, digest, plen);
		pos += plen;
		left -= plen;
	}
	return 0;
}


/**
 * @brief  ͨ��CPU_ID�����һ��������������MAC��ַ�ĵ�3�ֽ�
 *
 *
 */
int create_mac(unsigned char *mac)
{
	char psk[33], id[12];
	const char ps[] = "123456789";

	int i, j, k, ret;
	i =  *CPU_ID;
	j = *(CPU_ID + 1);
	k = *(CPU_ID + 2);
	memcpy(id, &i, 4);
	memcpy(id + 4, &j, 4);
	memcpy(id + 8, &k, 4);
	ret = pbkdf2_sha1(ps, id, 12, 100, (u8*)psk, 32);

	//ȡ�����ֽ���Ϊmac��ַ
	mac[0] = 0x00;
	mac[1] = 0x0c;
	mac[2] = 0x43;
	memcpy(mac + 3, psk, 3);
	mac[5] = mac[5] &0xfe; //for 2-BSSID mode

	dump_hex("mac", mac, 6);

	return ret;
}

extern int rtusb_probe(struct usb_device *dev);
extern int rt28xx_open(VOID *dev);
extern _wireless_event_callback wireless_event_callback;
int init_wifi()
{
	int ret;

	wireless_event_callback = event_callback;

	set_if_type(p_ad, INT_WDS);

	ret = rtusb_probe(&usb_dev);
	p_dbg("probe ret %d\n", ret);

	ret = rt28xx_open(usb_dev.net_dev);
	p_dbg("open ret %d\n", ret);


	return ret;
}
