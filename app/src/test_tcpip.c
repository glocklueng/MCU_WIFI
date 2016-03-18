#define DEBUG
#include "lwip\sockets.h"
#include "lwip\netif.h"
#include "lwip\dns.h"
#include "lwip\api.h"
#include "tcpapp.h"
#include "audio\audio.h"
#include "wifi.h"
#include "debug.h"
#include "lwip/dhcp.h"
#include "dhcpd.h"

//ȫ�ֱ�������
extern int errno; //lwip�����
extern mutex_t socket_mutex;

//��������
void test_tcp_recv(void *_fd);
int close_socket(uint32_t socket_num);

//socket �ļ����
int client_socket_fd =  - 1;
int server_socket_fd =  - 1;
int remote_socket_fd =  - 1;

struct sockaddr udp_remote_client; //���ڱ���Զ��udp�ͻ�����Ϣ

//thread ���
int server_accept_thread_fd =  - 1;

fd_set rfds;

/*
 * @brief  ���ӵ�IP��ַΪ192.168.1.101���˿ں�4700��TCP������
 *
 */
void test_tcp_link()
{
	char *ip = "192.168.1.101";
	uint16_t port = 4700;
	p_dbg_enter;
	p_dbg("���ӵ�:%s, �˿ں�:%d", ip, port);
	if (!is_wifi_connected())
	{

		p_err("wifi not connect");
		return ;
	}

	client_socket_fd = tcp_link(ip, port);

	p_dbg_exit;
}

/*
 * @brief  ���ӵ�IP��ַΪ192.168.1.101���˿ں�4701��UDP������
 *
 */
void test_udp_link()
{
	char *ip = "192.168.1.101";
	uint16_t port = 4701;
	p_dbg_enter;
	p_dbg("���ӵ�:%s, �˿ں�:%d", ip, port);
	if (!is_wifi_connected())
	{

		p_err("wifi not connect");
		return ;
	}

	client_socket_fd = udp_link(ip, port);

	p_dbg_exit;
}

/*
 * @brief  ��ӵ��ಥ��
 * �鲥��ַΪ224.0.0.1���˿ں�4702
 */
void test_multicast_join()
{
	char *ip = "224.0.0.1";
	uint16_t port = 4702;
	p_dbg_enter;
	p_dbg("���ӵ�:%s, �˿ں�:%d", ip, port);
	if (!is_wifi_connected())
	{

		p_err("wifi not connect");
		return ;
	}

	client_socket_fd = udp_add_membership(ip, port);

	p_dbg_exit;
}

/*
 * @brief  ����udp������
 * �˿ں�4703
 */
void test_udp_server()
{
	uint16_t port = 4703;
	p_dbg_enter;
	p_dbg("test_udp_server, �˿ں�:%d", port);
	if (!is_wifi_connected())
	{

		p_err("wifi not connect");
		return ;
	}

	server_socket_fd = udp_create_server(port);

	p_dbg_exit;
}


/*
 * @brief  �Ͽ�TCP����
 *
 */
void test_tcp_unlink()
{
	p_dbg_enter;
	if (client_socket_fd !=  - 1)
	{
		close_socket(client_socket_fd);
		client_socket_fd =  - 1;
	}
	p_dbg_exit;
}

/*
 * @brief  �������ݵ�Զ�̷�����
 *
 */
DECLARE_MONITOR_ITEM("tcp totol send", tcp_totol_send);


void test_send(char *pstr)
{
	int ret, len = strlen(pstr);
	p_dbg_enter;
	if (remote_socket_fd !=  - 1)
	{
		ret = send(remote_socket_fd, pstr, len, 0);
		if (ret != len)
		{
			p_err("send data err:%d", ret);
			close_socket(remote_socket_fd);
			remote_socket_fd =  - 1;
		}
	}
	if (client_socket_fd !=  - 1)
	{
		ret = send(client_socket_fd, pstr, len, 0);
		if (ret != len)
		{
			p_err("send data er1r:%d", ret);
			close_socket(client_socket_fd);
			client_socket_fd =  - 1;
		}
	}
	ADD_MONITOR_VALUE(tcp_totol_send, len);
	p_dbg_exit;
}

/*
 *������Է���udp���ݵ�Զ�̶�
 *fd ʹ��server_socket_fd����test_udp_server����������socket
 */
void test_sendto(char *pstr)
{
	struct lwip_sock *sock;
	struct sockaddr_in addr;
	uint32_t remote_addr;
	uint16_t remote_port;
	int ret, len = strlen(pstr);
	p_dbg_enter;
	if (server_socket_fd ==  - 1)
		return ;

	memcpy(&addr, &udp_remote_client, sizeof(struct sockaddr_in));
	remote_addr = addr.sin_addr.s_addr;
	remote_port = addr.sin_port;

	sock = get_socket(server_socket_fd);
	if (!sock || !sock->conn)
	{
		return ;
	}

	if (sock->conn->type != NETCONN_UDP)
	{
		p_err("is not udp socket");
		return ;
	}
	p_dbg("�������ݵ�:%x, �˿ں�:%d", remote_addr, remote_port); //������ʾ�Ĵ��ģʽ����ֵ

	udp_data_send(server_socket_fd, pstr, len, remote_port, remote_addr);

	p_dbg_exit;
}


/*
 * @brief  �رձ��ط�����
 *
 */
void test_close_tcp_server()
{
	p_dbg_enter;
	if (remote_socket_fd !=  - 1)
	{
		close_socket(remote_socket_fd);
		remote_socket_fd =  - 1;
	}

	if (server_socket_fd !=  - 1)
	{
		close_socket(server_socket_fd);
		server_socket_fd =  - 1;
	}

	if (server_accept_thread_fd !=  - 1)
	{
		thread_exit(server_accept_thread_fd);
		server_accept_thread_fd =  - 1;
	}

	p_dbg_exit;
}

/*
 * @brief ���ط����������߳�
 *
 */
void tcp_accept_task(void *server_fd)
{
	int sockaddr_len, new_socket, opt;
	struct sockaddr_in addr;

	sockaddr_len = sizeof(struct sockaddr);

	while (1)
	{
		p_dbg("waiting for remote connect");
		new_socket = accept((int)server_fd, (struct sockaddr*) &addr, (socklen_t*) &sockaddr_len);
		if (new_socket ==  - 1)
		{
			p_err("accept err");
			break;
		} p_dbg("accept a new client");
		remote_socket_fd = new_socket;

		opt = 1;
		if (setsockopt(new_socket, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(int)) ==  - 1)
			p_err("SO_KEEPALIVE err\n");

	}
	thread_exit(server_accept_thread_fd);
}


void test_dns(char *hostname)
{
	uint32_t addr;
	int ret;

	p_dbg_enter;
	ret = get_host_by_name(hostname, &addr);

	if (ret == 0)
	{
		p_dbg("get %s, ipaddr:: %d.%d.%d.%d\n", hostname, ip4_addr1(&addr), ip4_addr2(&addr), ip4_addr3(&addr), ip4_addr4(&addr));

	}
	p_dbg_exit;
}

/*
 * @brief  �ڱ��ؽ���һ�����������ȴ�����
 * �������˿ں�4800
 *
 */
void test_tcp_server()
{

	int socket_s =  - 1, err = 0;
	uint16_t port = 4800;
	struct sockaddr_in serv;
	memset(&serv, 0, sizeof(struct sockaddr_in));
	p_dbg_enter;

	p_dbg("����������, �˿ں�:%d", port);
	test_close_tcp_server();

	serv.sin_family = AF_INET;
	serv.sin_port = htons(port);
	serv.sin_addr.s_addr = htons(INADDR_ANY);

	socket_s = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_s ==  - 1)
	{
		goto err;
	}
	err = bind(socket_s, (struct sockaddr*) &serv, sizeof(struct sockaddr_in));
	if (err ==  - 1)
	{
		goto err;
	} err = listen(socket_s, 4);
	if (err ==  - 1)
	{
		goto err;
	}

	server_socket_fd = socket_s;

	server_accept_thread_fd = thread_create(tcp_accept_task, (void*)server_socket_fd, TASK_TCP_ACCEPT_PRIO, 0, TASK_ACCEPT_STACK_SIZE, "tcp_accept_task");

	return ;
	err: if (err < 0)
		p_err("err:%d", err);
	if (socket_s !=  - 1)
		close_socket(socket_s);
	p_dbg_exit;
}


void show_tcpip_info()
{
	ip_addr_t dns_server;

	dns_server = dns_getserver(0);

	p_dbg("ipaddr: %d.%d.%d.%d", ip4_addr1(&p_netif->ip_addr.addr), ip4_addr2(&p_netif->ip_addr.addr), ip4_addr3(&p_netif->ip_addr.addr), ip4_addr4(&p_netif->ip_addr.addr));

	p_dbg("netmask: %d.%d.%d.%d", ip4_addr1(&p_netif->netmask.addr), ip4_addr2(&p_netif->netmask.addr), ip4_addr3(&p_netif->netmask.addr), ip4_addr4(&p_netif->netmask.addr));

	p_dbg("gw: %d.%d.%d.%d", ip4_addr1(&p_netif->gw.addr), ip4_addr2(&p_netif->gw.addr), ip4_addr3(&p_netif->gw.addr), ip4_addr4(&p_netif->gw.addr));


	p_dbg("dns_server: %d.%d.%d.%d", ip4_addr1(&dns_server.addr), ip4_addr2(&dns_server.addr), ip4_addr3(&dns_server.addr), ip4_addr4(&dns_server.addr));
}


/*
 * @brief  �Զ���ȡIP����
 *
 *
 */
void test_auto_get_ip()
{
	int i, wait_time = 10;

	auto_get_ip();

	for (i = 0; i < wait_time; i++)
	{
		p_dbg("%d", i);
		if (p_netif->ip_addr.addr)
			break;
		sleep(1000);
	}

	if (p_netif->ip_addr.addr)
	{
		show_tcpip_info();
	}

}

/*
 * @brief  ����Ƶ����
 *
 */
void test_open_audio()
{
	int ret;
	ret = audio_dev_open();
	if (ret != 0)
	{
		p_err("open err");
		return ;
	}

	audio_cfg.volume = 100;
	audio_cfg.audio_net_port = 4700;
	audio_cfg.dec_input->samplerate = audio_cfg.adc->samplerate = AUDIO_SAMPLERATE32000;

	adc_switch_samplerate(audio_cfg.adc->samplerate);
	dac_switch_samplerate(audio_cfg.dec_input->samplerate);
}

/*
 * @brief  �ر���Ƶ����
 *
 */
void test_close_audio()
{
	audio_dev_close();
}


/*
 * @brief  ������Ƶ����
 * ��adc_recv_thread����
 */
void send_audio_data_to_remote(char *buff, int len)
{
	int ret;
	if (remote_socket_fd !=  - 1)
	{
		ret = send(remote_socket_fd, buff, len, 0);
		if (ret != len)
		{
			p_err("send data err:%d", ret);
			close_socket(remote_socket_fd);
			remote_socket_fd =  - 1;
		}
	}
	if (client_socket_fd !=  - 1)
	{
		ret = send(client_socket_fd, buff, len, 0);
		if (ret != len)
		{
			p_err("send data er1r:%d", ret);
			close_socket(client_socket_fd);
			client_socket_fd =  - 1;
		}
	}
	ADD_MONITOR_VALUE(tcp_totol_send, len);
}

int close_socket(uint32_t socket_num)
{
	struct lwip_sock *sock;
	int ret;
	p_dbg_enter;
	sock = get_socket(socket_num);

	if (!sock || !sock->conn)
	{
		p_err("close_socket err1\n");
		return  - 1;
	} if (!((sock->conn->state != NETCONN_CONNECT) || (NETCONN_FLAG_IN_NONBLOCKING_CONNECT &(sock->conn->flags))))
	{
		p_err("close_socket err2\n");
		return  - 1;
	}
	mutex_lock(socket_mutex);
	shutdown(socket_num, SHUT_RDWR);
	ret = close(socket_num);
	if (ret ==  - 1)
		p_err("close_socket err4:%d\n", ret);
	if (FD_ISSET(socket_num, &rfds))
		FD_CLR(socket_num, &rfds);
	mutex_unlock(socket_mutex);
	p_dbg_exit;
	return ret;
}

int is_led_ctrl_cmd(char *buff)
{
	if ((buff[0] == 0xaa) && (buff[1] == 0x55) && (buff[2] == 1 || buff[2] == 2))
		return 1;

	return 0;
}

/*�Ƿ���udpserver������������ǵĻ�����Ҫȡ��Զ�̶˵�ַ*/
int is_udp_server_socket(int num)
{
	uint16_t tmp_port = 0;
	struct lwip_sock *sock;

	if (num != server_socket_fd)
		return 0;

	sock = get_socket(num);
	if (!sock || !sock->conn)
	{
		return 0;
	}

	if (sock->conn->type == NETCONN_UDP)
	{
		return 1;
	}

	return 0;

}


int is_dhcp_socket(int num)
{
	uint16_t tmp_port = 0;
	struct lwip_sock *sock;

	sock = get_socket(num);
	if (!sock || !sock->conn)
	{
		return 0;
	}

	if (sock->conn->type == NETCONN_UDP)
	{
		tmp_port = sock->conn->pcb.udp->local_port; //server

		if (tmp_port == DHCP_SERVER_PORT)
			return 1;
	}
	return 0;
}

/*
 *selcetģʽ�������ݵ����ӣ����Լ�ض��socket�����ݽ���
 *���з��͵��������udp��tcp���ݶ��������ȡ
 */
DECLARE_MONITOR_ITEM("tcp totol recv", tcp_totol_recv);
#define TCP_RCV_SIZE 1024
void tcp_recv_thread(void *arg)
{
	int i, size, retval, select_size, udp_server_data;
	struct lwip_sock *sock;
	char *tcp_rcv_buff;
	struct timeval tv;

	tcp_rcv_buff = (char*)mem_malloc(TCP_RCV_SIZE);
	while (1)
	{
		mutex_lock(socket_mutex);
		FD_ZERO(&rfds);

		for (i = 0; i < MEMP_NUM_NETCONN; i++)
		{
			sock = get_socket(i);
			if (sock && sock->conn && sock->conn->recvmbox)
			//socket�رպͽ��չر����ܽ���
			{
				FD_SET(i, &rfds);
			}
		}
		select_size = MEMP_NUM_NETCONN;

		mutex_unlock(socket_mutex);

		tv.tv_sec = 5;
		tv.tv_usec = 0;
		retval = select(select_size, &rfds, NULL, NULL, &tv);
		if ((retval ==  - 1) || (retval == 0))
		{
			sleep(50); //�ڴ����
		}
		else
		{
			if (retval)
			{
				for (i = 0; i < select_size; i++)
				{
					if (FD_ISSET(i, &rfds))
					{
						struct sockaddr remote_addr;
						mutex_lock(socket_mutex);

						#if 1	//ʹ��recvfrom����
						retval = sizeof(struct sockaddr);
						size = recvfrom(i, tcp_rcv_buff, TCP_RCV_SIZE, MSG_DONTWAIT, &remote_addr, (socklen_t*) &retval);
						/*	
						p_dbg("rcv from:%d,%d,%d,%d,%d,%d\n",udp_remote_addr.sa_data[0],remote_addr.sa_data[1],
						remote_addr.sa_data[2],
						remote_addr.sa_data[3],
						remote_addr.sa_data[4],
						remote_addr.sa_data[5]	);*/
						#else //ʹ��recv����
						size = recv(i, tcp_rcv_buff, TCP_RCV_SIZE, MSG_DONTWAIT);
						#endif
						mutex_unlock(socket_mutex);
						if (size ==  - 1)
						{
							if (errno == EWOULDBLOCK || errno == ENOMEM /* ||errno == EINTR*/)
							{
								p_err("tcp_recv err:%d,%d\n", i, errno);
								sleep(10);
							}
							else
							{
								p_err("tcp_recv fatal err:%d,%d\n", i, errno);
								close_socket(i);
							}
							continue;
						}
						if (size == 0)
						//0���������Ѿ��ر�
						{
							if (errno != 0)
							{
								p_err("tcp_client_recv err1:%d\n", errno);
								close_socket(i);
							}
							else
								p_err("rcv 0 byte?\n");
							continue;
						}

						//p_dbg("socket:%d rcv:%d\n", i, size);
						ADD_MONITOR_VALUE(tcp_totol_recv, size);

						if (is_udp_server_socket(i))
						{
							udp_remote_client = remote_addr;
							udp_server_data = 1;
						}
						else
							udp_server_data = 0;

						if (is_dhcp_socket(i))
						{

							handle_dhcp_rcv((uint8_t*)tcp_rcv_buff, size);
						}
						else
						{

							if (audio_cfg.audio_dev_open)
							{
								handle_audio_stream((unsigned char*)tcp_rcv_buff, size); //������Ƶ
							}
							else
							{

								if (is_led_ctrl_cmd(tcp_rcv_buff))
								{
									if (tcp_rcv_buff[2] == 1)
										led_switch(tcp_rcv_buff[3]);
									if (tcp_rcv_buff[2] == 2)
										led_bright(tcp_rcv_buff[3]);
								}

								//���ظ��Է�
								if (udp_server_data)
									sendto(i, (u8*)tcp_rcv_buff, size, 0, &udp_remote_client, sizeof(struct sockaddr));
								else
									send(i, tcp_rcv_buff, size, 0);


								ADD_MONITOR_VALUE(tcp_totol_send, size);
								//dump_hex("data", recv_buff, ret);//16������ʽ��ӡ����
								//p_dbg("%s", recv_buff);//�ַ�����ʽ��ӡ����
							}
						}
					} //end of if(FD_ISSET(i,&rfds))
				} //end of for(i = 0; i < select_size; i++)
			}
		}
	}
}
