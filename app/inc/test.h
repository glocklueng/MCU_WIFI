#ifndef _TEST_H
#define _TEST_H

void test_scan(void);
void test_wifi_connect(void);
void test_start_ap(void);
void test_wifi_disconnect(void);

void test_tcp_link(void);
void test_tcp_unlink(void);
void test_send(char *pstr);
void test_sendto(char *pstr);
void test_close_tcp_server(void);
void test_dns(char *hostname);
void test_tcp_server(void);
void test_auto_get_ip(void);
void test_open_audio(void);
void test_close_audio(void);
void test_wifi_get_stats(void);
void show_tcpip_info(void);
void show_sys_info(void);
void test_udp_link(void);
void test_multicast_join(void);
void test_udp_server(void);


#endif
