/*
 * WIFI配置的更多功能请参考net/rt5572.../module/common/ap_cfg.c文件
 *
 */


#define DEBUG
#include "wifi.h"
#include "drivers.h"
#include "tcpapp.h"

PRTMP_ADAPTER p_ad = 0;

/*
 * @brief  wifi连接完成的回调函数
 *
 */
void link_finish(void *ptmr, void *parg)
{
	p_dbg_enter;
}

/**
 * @brief  设置wifi模式
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

	lwip_netif_init(); //切换模式后需要重新初始化lwip网络设备，主要是mac地址的改变
}



/**
 * @brief  设置wifi频道
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
 * @brief  设置sta模式下的密码
 * @param  type : 取值如下
 * 						"WEP"  WEP加密
 * 						"WPA"	WPA或WPA2加密
 *				 key: 明文密码	
 * @retval ok: 0, err: -1
 */
int wifi_client_add_key(char *type, char *key)
{
	if (strcmp(type, "WEP") == 0)
	{
		Set_ApCli_Key1_Proc(p_ad, key);
		Set_ApCli_DefaultKeyID_Proc(p_ad, "1"); //为了节省内存，我们只用一个wepkey

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
 * @brief  设置wifi模式为AP参数
 *
 * @param  type : name:网络名称，key:密码
key_type
取值:KEY_NONE = 0
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

		Set_AP_SSID_Proc(p_ad, name); //Set_AP_WPAPSK_Proc要用到ssid,所以必须先调用

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
 * @brief  扫描附近的AP
 * @param
 * @retval ok: 0, err: -1
 */
BSS_ENTRY_SIMPLE *wifi_scan()
{
	Set_SiteSurvey_Proc(p_ad, ""); //第二个参数用于指定要搜索的essid，空置表示搜索全部
	//延时3s等待扫描完成,也可以等待事件通知，扫描完成事件号是0x0211
	sleep(3000);

	return BssTabCloneSimple(&p_ad->ScanTabSimple); //拷贝一份扫描到的结果
}


/**
 * @brief  wifi 是否已经连接
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
 * @brief  wifi连接
 * @param
 *							
 *				 essid: 即AP名称
 *				 key: 明文密码	
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
 * @brief  断开连接
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
 * @brief  读取mac地址，mac地址是在NICReadEEPROMParameters函数里面设置的，
 * 用户可以在里面更改为任意地址
 * @param  mac_addr : mac地址 6BYTE
 */
int wifi_get_mac(unsigned char *mac_addr)
{
	memcpy(mac_addr, p_ad->CurrentAddress, MAC_ADDR_LEN);
	return 0;
}


/**
 * @brief  获取WIFI连接的统计信息(信号强度...)
 *
 * @param  pStats : 指向iw_statistics的指针
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
 * @brief  WIFI 事件回调函数,包括扫描完成，连接断开等事件通知
 * @param  type :取值如下
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
//sha1散列算法
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
 * @brief  通过CPU_ID计算出一个随机数用于填充MAC地址的低3字节
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

	//取三个字节作为mac地址
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
