#define DEBUG
#include "wifi.h"
#include "dhcpd.h"


void test_scan()
{
	int i = 0;
	BSS_TABLE_SIMPLE table;
	BSS_ENTRY_SIMPLE *p_bss_entry;

	p_bss_entry = wifi_scan();

	//结果是以链表保存的
	if (p_bss_entry)
	{
		table.BssEntry = p_bss_entry;
		do
		{
			p_dbg("%d: %s\r\n", i++, p_bss_entry->res_data.essid);
			/*
			p_dbg(" mode:%d", p_bss_entry->res_data.mode);
			p_dbg(" proto:%d", p_bss_entry->res_data.proto);
			p_dbg(" cipher:%d", p_bss_entry->res_data.cipher);
			p_dbg(" rssi:%d", p_bss_entry->res_data.rssi);*/
			p_dbg(" channel:%d", p_bss_entry->res_data.channel);
			dump_hex("	mac", p_bss_entry->res_data.bssid, 6);
			p_bss_entry = p_bss_entry->next;
		}
		while (p_bss_entry);

		BssTabClrSimple(&table)
			;
		//将拷贝出来的结果释放
	}

	p_dbg("find %d bss", i);
}

/**
 *测试WIFI连接
 *连接到名字为"LCK”的路由器，密码是"ckxr1314”，路由器加密模式为WPA2PSK+AES
 *密码长度在WPA或WPA2模式下8 <= len <= 64;在WEP模式下必须为5或13
 */
void test_wifi_connect()
{
	char channel[8] = "6";
	BSS_TABLE_SIMPLE table;
	BSS_ENTRY_SIMPLE *p_bss_entry;
	int delay_time = 10;
	char *essid = "LCK";
	char *password = "WXRLXRLCK";
	p_dbg_enter;

	p_bss_entry = wifi_scan();
	if (p_bss_entry)
	{
		table.BssEntry = p_bss_entry;
		do
		{
			if(strcmp(p_bss_entry->res_data.essid, essid) == 0)
			{
				sprintf(channel, "%d", p_bss_entry->res_data.channel);
				p_dbg("get target ap's channel:%d", p_bss_entry->res_data.channel);
				break;
			}
			p_bss_entry = p_bss_entry->next;
		}
		while (p_bss_entry);
		BssTabClrSimple(&table);
	}

	p_dbg("连接到:%s, 密码:%s", essid, password);

	wifi_set_mode(INT_APCLI);
	wifi_connect("WPA2PSK", "AES", essid, password, channel);
	//wifi_connect("OPEN", "NONE", essid, "");

	while (delay_time--)
	{
		sleep(1000);
		if (is_wifi_connected())
			break;
	}

	if (is_wifi_connected())
	{
		p_dbg("wifi connect ok");
	}

	p_dbg_exit;
}


/*
 *建立AP
 *
 */
void test_start_ap()
{
	char *essid = "lck123";
	char *password = "12345678";
	p_dbg_enter;
	p_dbg("建立AP:%s, 密码:%s", essid, password);

	wifi_ap_cfg(essid, password, KEY_WPA2, 6, 0);
	p_dbg_exit;
}

/*
 *测试断开WIFI连接
 *
 */
void test_wifi_disconnect()
{
	p_dbg_enter;
	wifi_disconnect();
	p_dbg_exit;
}

/*
 *测试获取WIFI连接信息
 *
 */
void test_wifi_get_stats()
{
	struct iw_statistics stats;
	int rssi;
	char rssi1;

	wifi_get_stats(&stats);

	p_dbg("wifi stats, qual:%d, rssi:%d", stats.qual.qual, stats.qual.level);
}
