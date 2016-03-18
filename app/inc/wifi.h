#ifndef _WIFI_H_
#define _WIFI_H_

#include "rt_config.h"

typedef enum
{
	KEY_NONE = 0, KEY_WEP, KEY_WPA, KEY_WPA2, KEY_MAX_VALUE
} SECURITY_TYPE;

void link_finish(void *ptmr, void *parg);
int wifi_set_mode(int type);
int wifi_client_add_key(char *type, char *key);
BSS_ENTRY_SIMPLE *wifi_scan(void);
int is_wifi_connected(void);
int wifi_disconnect(void);int wifi_connect(char *authmode, char *encryp, char *essid, char *key, char *channel);

int wifi_get_mac(unsigned char *mac_addr);
int init_wifi(void);
int wifi_set_channel(int channel);
int wifi_get_stats(struct iw_statistics *pStats);
int wifi_ap_cfg(char *name, char *key, int key_type, int channel, int hidden);

extern PRTMP_ADAPTER p_ad;

#endif
