#ifndef __RT3070_USER_H__
#define __RT3070_USER_H__

#include "bsp.h"
#include "debug.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/arch/sys_arch.h"
#include "rtmp_type.h"
#include "mac80211_def.h"

#define KERN_WARNING
#define IN
#define OUT
#define INOUT
#define NDIS_STATUS		INT

#define mdelay sleep

#define IW_PRIV_TYPE_MASK	0x7000	/* Type of arguments */
#define IW_PRIV_TYPE_NONE	0x0000
#define IW_PRIV_TYPE_BYTE	0x1000	/* Char as number */
#define IW_PRIV_TYPE_CHAR	0x2000	/* Char as character */
#define IW_PRIV_TYPE_INT	0x4000	/* 32 bits int */
#define IW_PRIV_TYPE_FLOAT	0x5000	/* struct iw_freq */
#define IW_PRIV_TYPE_ADDR	0x6000	/* struct sockaddr */

#define IW_PRIV_SIZE_FIXED	0x0800	/* Variable or fixed number of args */

#define IW_PRIV_SIZE_MASK	0x07FF	/* Max number of those args */

#define ARPHRD_IEEE80211_PRISM 802

#define __bitwise
#define __force


#define WPA_PROTO_NONE 	0
#define WPA_PROTO_WPA 	1		//BIT(0)
#define WPA_PROTO_RSN 	2		//BIT(1)
#define WPA_PROTO_OPEN_SHARE	3


#define WPA_AUTH_ALG_OPEN BIT(0)
#define WPA_AUTH_ALG_SHARED BIT(1)
#define WPA_AUTH_ALG_LEAP BIT(2)
#define WPA_AUTH_ALG_FT BIT(3)


#define	IFNAMSIZ	16
#define	IFALIASZ	256

#if 	1		//MT7601/RTxxxx开关
#define RT30xx				
#define RT33xx			
#define RT3070	
#define RT3370
#define RT5370
#define RT5390
#else
#define MT7601
#endif
//#define RTMP_FLASH_SUPPORT 

#define INIT

#ifdef MT7601
#define CONFIG_ANDES_SUPPORT
#define RLT_RF					
#define RLT_MAC			
#define CONFIG_RX_CSO_SUPPORT
#define NEW_MBSSID_MODE
//#else
//#define RTMP_MAC				
#endif

//#define RTMP_TEMPERATURE_COMPENSATION
//#define IQ_CAL_SUPPORT
//#define VCORECAL_SUPPORT

#define USE_ONLY_ONE_KEY				0
#define USE_SIMPLE_TRANSMIT 			0
#define USE_14_CHANNELS				1
//#define WSC_AP_SUPPORT			//wps
//#define WSC_LED_SUPPORT
#define SYSTEM_LOG_SUPPORT		//事件通知
#define UAPSD_SUPPORT				//省电模式
#define LED_CONTROL_SUPPORT
#define RTMP_EFUSE_SUPPORT
#define MLMEQUEUE_ALLOC_INUSE		1
#define RTMP_TIMER_TASK_SUPPORT		1
#define RTMP_RF_RW_SUPPORT			1
//#define OS_ABL_FUNC_SUPPORT			1
//#define RT_CFG80211_SUPPORT			1
#define OS_ABL_OS_STA_SUPPORT		1	
#define OS_ABL_OS_AP_SUPPORT			1
#define CONFIG_AP_SUPPORT				1
#define CONFIG_STA_SUPPORT			1
//#define CONFIG_APSTA_MIXED_SUPPORT
#define	MAT_SUPPORT
//#define OS_ABL_SUPPORT				1
#define DOT11_N_SUPPORT				1	//80211n
#define DOT11N_DRAFT3					1	//80211n
#define WORKQUEUE_BH					1
#define IW_HANDLER_VERSION				0
#define WIRELESS_EXT					22
#define LINUX							1
#define RTMP_INTERNAL_TX_ALC			1
#define MEMORY_OPTIMIZATION			1

#define RTMP_USB_SUPPORT
#define RTMP_MAC_USB
#define WPA_SUPPLICANT_SUPPORT
#define AP_SCAN_SUPPORT
#define APCLI_SUPPORT


#define IW_CUSTOM_MAX 256

#define ETH_P_EAPOL ETH_P_PAE
/* ----------------------- WIRELESS EVENTS ----------------------- */
/* Those are *NOT* ioctls, do not issue request on them !!! */
/* Most events use the same identifier as ioctl requests */

#define IWEVTXDROP	0x8C00		/* Packet dropped to excessive retry */
#define IWEVQUAL	0x8C01		/* Quality part of statistics (scan) */
#define IWEVCUSTOM	0x8C02		/* Driver specific ascii string */
#define IWEVREGISTERED	0x8C03		/* Discovered a new node (AP mode) */
#define IWEVEXPIRED	0x8C04		/* Expired a node (AP mode) */
#define IWEVGENIE	0x8C05		/* Generic IE (WPA, RSN, WMM, ..)
					 * (scan results); This includes id and
					 * length fields. One IWEVGENIE may
					 * contain more than one IE. Scan
					 * results may contain one or more
					 * IWEVGENIE events. */
#define IWEVMICHAELMICFAILURE 0x8C06	/* Michael MIC failure
					 * (struct iw_michaelmicfailure)
					 */
#define IWEVASSOCREQIE	0x8C07		/* IEs used in (Re)Association Request.
					 * The data includes id and length
					 * fields and may contain more than one
					 * IE. This event is required in
					 * Managed mode if the driver
					 * generates its own WPA/RSN IE. This
					 * should be sent just before
					 * IWEVREGISTERED event for the
					 * association. */
#define IWEVASSOCRESPIE	0x8C08		/* IEs used in (Re)Association
					 * Response. The data includes id and
					 * length fields and may contain more
					 * than one IE. This may be sent
					 * between IWEVASSOCREQIE and
					 * IWEVREGISTERED events for the
					 * association. */
#define IWEVPMKIDCAND	0x8C09		/* PMKID candidate for RSN
					 * pre-authentication
					 * (struct iw_pmkid_cand) */
/*
#define (LINUX_VERSION_CODE >= KERNEL_VERSION(a,b,c)) 	1
#define (LINUX_VERSION_CODE > KERNEL_VERSION(a,b,c)) 	1

#define (LINUX_VERSION_CODE <= KERNEL_VERSION(a,b,c)) 	0
#define (LINUX_VERSION_CODE < KERNEL_VERSION(a,b,c)) 	0*/


#include "rtmp_type.h"
#include "mac80211_def.h"


typedef struct _LIST_HEADR
{
	PLIST_ENTRY pHead;
	PLIST_ENTRY pTail;
	UCHAR size;
} LIST_HEADER, *PLIST_HEADER;

VOID initList(
	IN PLIST_HEADER pList);

VOID insertTailList(
	IN PLIST_HEADER pList,
	IN PLIST_ENTRY pEntry);

PLIST_ENTRY removeHeadList(
	IN PLIST_HEADER pList);

int getListSize(
	IN PLIST_HEADER pList);

PLIST_ENTRY delEntryList(
	IN PLIST_HEADER pList,
	IN PLIST_ENTRY pEntry);



struct	iw_quality
{
	__u8		qual;		/* link quality (%retries, SNR,
					   %missed beacons or better...) */
	__u8		level;		/* signal level (dBm) */
	__u8		noise;		/* noise level (dBm) */
	__u8		updated;	/* Flags to know if updated */
};

struct	iw_missed
{
	__u32		beacon;		/* Missed beacons/superframe */
};

struct	iw_discarded
{
	__u32		nwid;		/* Rx : Wrong nwid/essid */
	__u32		code;		/* Rx : Unable to code/decode (WEP) */
	__u32		fragment;	/* Rx : Can't perform MAC reassembly */
	__u32		retries;	/* Tx : Max MAC retries num reached */
	__u32		misc;		/* Others cases */
};


struct	iw_statistics
{
	__u16		status;		/* Status
					 * - device dependent for now */

	struct iw_quality	qual;		/* Quality of the link
						 * (instant/mean/max) */
	struct iw_discarded	discard;	/* Packet discarded counts */
	struct iw_missed	miss;		/* Packet missed counts */
};

typedef int (*iw_handler)(struct net_device *dev, void *info,
			  void *wrqu, char *extra);

struct work_struct {
	LIST_HEADER * pTaskletList;
	LIST_ENTRY entry;
	VOID (*fun) (void *data);
	void	*data;
	OS_EVENT *event;
};

struct ifreq {
#define IFHWADDRLEN	6
	union
	{
		char	ifrn_name[IFNAMSIZ];		/* if name, e.g. "en0" */
	} ifr_ifrn;
	
	union {
		struct	sockaddr ifru_addr;
		struct	sockaddr ifru_dstaddr;
		struct	sockaddr ifru_broadaddr;
		struct	sockaddr ifru_netmask;
		struct  sockaddr ifru_hwaddr;
		short	ifru_flags;
		int	ifru_ivalue;
		int	ifru_mtu;
//		struct  ifmap ifru_map;
		char	ifru_slave[IFNAMSIZ];	/* Just fits the size */
		char	ifru_newname[IFNAMSIZ];
//		void __user *	ifru_data;
//		struct	if_settings ifru_settings;
	} ifr_ifru;
};

#define URB_SHORT_NOT_OK	0x0001	/* report short reads as errors */
#define URB_ISO_ASAP		0x0002	/* iso-only, urb->start_frame
					 * ignored */
#define URB_NO_TRANSFER_DMA_MAP	0x0004	/* urb->transfer_dma valid on submit */
#define URB_NO_FSBR		0x0020	/* UHCI-specific */
#define URB_ZERO_PACKET		0x0040	/* Finish bulk OUT with short packet */
#define URB_NO_INTERRUPT	0x0080	/* HINT: no non-error interrupt
					 * needed */
#define URB_FREE_BUFFER		0x0100	/* Free transfer buffer with the URB */

/* The following flags are used internally by usbcore and HCDs */
#define URB_DIR_IN		0x0200	/* Transfer from device to host */
#define URB_DIR_OUT		0
#define URB_DIR_MASK		URB_DIR_IN

#define URB_DMA_MAP_SINGLE	0x00010000	/* Non-scatter-gather mapping */
#define URB_DMA_MAP_PAGE	0x00020000	/* HCD-unsupported S-G */
#define URB_DMA_MAP_SG		0x00040000	/* HCD-supported S-G */
#define URB_MAP_LOCAL		0x00080000	/* HCD-local-memory mapping */
#define URB_SETUP_MAP_SINGLE	0x00100000	/* Setup packet DMA mapped */
#define URB_SETUP_MAP_LOCAL	0x00200000	/* HCD-local setup packet */
#define URB_DMA_SG_COMBINED	0x00400000	/* S-G entries were combined */
#define URB_ALIGNED_TEMP_BUFFER	0x00800000	/* Temp buffer was alloc'd */


#define os_alloc_mem(X, Y, Z)	(*(PUCHAR*)(Y) = (PUCHAR)kmalloc((Z), GFP_ATOMIC),\
		(*(PUCHAR*)(Y) == 0)?NDIS_STATUS_FAILURE:NDIS_STATUS_SUCCESS)



#define EXPORT_SYMBOL_GPL(x);	

#define msleep	mdelay

#define usleep	delay_us

#define udelay	delay_us

#define test_bit(nr, addr)   (*addr & (1L << nr))

#define __set_bit(nr, addr)   *addr |=  (1L << nr)
#define __clr_bit(nr, addr)   *addr &=  ~(1L << nr)

#define set_bit __set_bit
#define clear_bit __clr_bit

#define msecs_to_jiffies(x)	(x/10)

#define 	BUG_ON(x) 			assert_param(!(x))

#define 	WARN_ON(x) 		assert_param(!(x))
#define 	BUILD_BUG_ON(x)	assert_param(!(x))

//#define RTMP_OS_MAX_SCAN_DATA_GET		RtmpOsMaxScanDataGet
#define vmalloc						mem_malloc
#define vfree							mem_free
#define simple_strtol					strtol

extern int copy_from_user(void *dst, void *src, int size) ;

#define copy_to_user  copy_from_user

//#ifdef DBG
#define printk		  printf
//#else
//#define printk(...)	 
//#endif
/*
#define snprintf(a,b, ...)					do{	\
	p_dbg("snprintf size:%d\n", b);				\
	sprintf(a, __VA_ARGS__);						\
	}while(0)
*/

/*do{	\
	memcpy(a, c, (strlen(c) > b)?b:(strlen(c)));	\
}while(0)*/

#define IW_SCAN_MAX_DATA			4096
#define KBUILD_MODNAME 			"wsum"

/*
#define skb_padto		p_err("%s miss\n", __FUNCTION__)
#define skb_pull(x,y) 	       p_err("%s miss\n", __FUNCTION__)
#define skb_put(x,y) 	       p_err("%s miss\n", __FUNCTION__)
#define skb_push(x,y) 	p_err("%s miss\n", __FUNCTION__)
#define skb_pad(x,y) 		p_err("%s miss\n", __FUNCTION__)
#define skb_trim(x,y)			p_err("%s miss\n", __FUNCTION__)
*/

#define kmalloc(x,y) mem_malloc(x)

#define GFP_KERNEL	0
#define GFP_ATOMIC 	0

#define kcalloc(x,y,z)	mem_calloc(x, y)
#define kzalloc(x,y) mem_calloc(x, 1)
#define kfree		mem_free

#define ieee80211_start_tx_ba_cb_irqsafe(X,Y,Z) p_err("%s miss\n", __FUNCTION__)
#define ieee80211_stop_tx_ba_cb_irqsafe(X,Y,Z)  p_err("%s miss\n", __FUNCTION__)

#define inline __inline

#define unlikely
#define likely

#define  is_broadcast_ether_addr(addr)  (((addr)[0] & (addr)[1] & (addr)[2] & (addr)[3] & (addr)[4] & (addr)[5]) == 0xff)

#define  is_zero_ether_addr(addr)  (!((addr)[0] | (addr)[1] | (addr)[2] | (addr)[3] | (addr)[4] | (addr)[5]))

#define  is_multicast_ether_addr(addr)  (0x01 & (addr)[0])

#define  is_local_ether_addr(addr)  (0x02 & (addr)[0])

#define  is_unicast_ether_addr(addr)	!(is_multicast_ether_addr)

#define  is_valid_ether_addr(addr)	 (!is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr))

#define clamp_t(A,B,C,D)	B

#define  random_ether_addr(addr)  do{		\
	addr[0] = jiffies;			\
	addr[1] = jiffies>>1;		\
	addr[2] = jiffies>>2;		\
	addr[3] = jiffies>>3;		\
	addr[4] = jiffies>>4;		\
	addr[5] = jiffies>>5;		\
	addr [0] &= 0xfe;			\
	addr [0] |= 0x02;			\
}while(0)

typedef u32 dma_addr_t;


#define CONFIG_RT2X00_LIB_CRYPTO 		1
#define CONFIG_RT2X00_LIB_FIRMWARE	1

#define  MODULE_AUTHOR(X,Y);
#define  MODULE_VERSION(X);
#define  MODULE_DESCRIPTION(X);
#define  MODULE_LICENSE(X);
#define  MODULE_SUPPORTED_DEVICE(X);
#define  MODULE_DEVICE_TABLE(x, y);
#define  MODULE_FIRMWARE(x);
#define  module_param_named(A, B, C, D);
#define  MODULE_PARM_DESC(A, B);
#define module_param(A,B,C);

typedef unsigned char *sk_buff_data_t;

typedef enum {
	SS_FREE = 0,			/* not allocated		*/
	SS_UNCONNECTED,			/* unconnected to any socket	*/
	SS_CONNECTING,			/* in process of connecting	*/
	SS_CONNECTED,			/* connected to socket		*/
	SS_DISCONNECTING		/* in process of disconnecting	*/
} socket_state;

struct socket {
	socket_state		state;

//	kmemcheck_bitfield_begin(type);
	short			type;
//	kmemcheck_bitfield_end(type);

	unsigned long		flags;

//	struct socket_wq __rcu	*wq;

	struct file		*file;
//	struct sock		*sk;
//	const struct proto_ops	*ops;
};
#if 0
struct sk_buff {
	/* These two members must be first. */
	struct sk_buff		*next;
	struct sk_buff		*prev;

//	ktime_t			tstamp;

	struct sock		*sk;
	struct net_device	*dev;

	/*
	 * This is the control buffer. It is free to use for every
	 * layer. Please put your private variables there. If you
	 * want to keep them across layers you have to do a skb_clone()
	 * first. This is owned by whoever has the skb queued ATM.
	 */
	char			cb[48]/* __aligned(8)*/;

	unsigned long		_skb_refdst;
#ifdef CONFIG_XFRM
	struct	sec_path	*sp;
#endif
	unsigned int		len,
				data_len;
	u16			mac_len,
				hdr_len;
	union {
		uint32_t		csum;
		struct {
			u16	csum_start;
			u16	csum_offset;
		};
	};
	__u32			priority;
//	kmemcheck_bitfield_begin(flags1);
	__u8			local_df:1,
				cloned:1,
				ip_summed:2,
				nohdr:1,
				nfctinfo:3;
	__u8			pkt_type:3,
				fclone:2,
				ipvs_property:1,
				peeked:1,
				nf_trace:1;
//	kmemcheck_bitfield_end(flags1);
	__be16			protocol;

	void			(*destructor)(struct sk_buff *skb);
#if defined(CONFIG_NF_CONNTRACK) || defined(CONFIG_NF_CONNTRACK_MODULE)
	struct nf_conntrack	*nfct;
#endif
#ifdef NET_SKBUFF_NF_DEFRAG_NEEDED
	struct sk_buff		*nfct_reasm;
#endif
#ifdef CONFIG_BRIDGE_NETFILTER
	struct nf_bridge_info	*nf_bridge;
#endif

	int			skb_iif;
#ifdef CONFIG_NET_SCHED
	__u16			tc_index;	/* traffic control index */
#ifdef CONFIG_NET_CLS_ACT
	__u16			tc_verd;	/* traffic control verdict */
#endif
#endif

	__u32			rxhash;

	u16			queue_mapping;
//	kmemcheck_bitfield_begin(flags2);
#ifdef CONFIG_IPV6_NDISC_NODETYPE
	__u8			ndisc_nodetype:2;
#endif
	__u8			ooo_okay:1;
//	kmemcheck_bitfield_end(flags2);

	/* 0/13 bit hole */

#ifdef CONFIG_NET_DMA
	dma_cookie_t		dma_cookie;
#endif
#ifdef CONFIG_NETWORK_SECMARK
	__u32			secmark;
#endif
	union {
		__u32		mark;
		__u32		dropcount;
	};

	u16			vlan_tci;

	sk_buff_data_t		transport_header;
	sk_buff_data_t		network_header;
	sk_buff_data_t		mac_header;
	/* These elements must be at the end, see alloc_skb() for details.  */
	sk_buff_data_t		tail;
	sk_buff_data_t		end;
	unsigned char		*head,
				*data;
	unsigned int		truesize;
	atomic_t		users;
};
#endif
#define USB_ENDPOINT_NUMBER_MASK	0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK		0x80

#define USB_ENDPOINT_SYNCTYPE		0x0c
#define USB_ENDPOINT_SYNC_NONE		(0 << 2)
#define USB_ENDPOINT_SYNC_ASYNC		(1 << 2)
#define USB_ENDPOINT_SYNC_ADAPTIVE	(2 << 2)
#define USB_ENDPOINT_SYNC_SYNC		(3 << 2)

#define USB_ENDPOINT_XFERTYPE_MASK	0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL	0
#define USB_ENDPOINT_XFER_ISOC		1
#define USB_ENDPOINT_XFER_BULK		2
#define USB_ENDPOINT_XFER_INT		3
#define USB_ENDPOINT_MAX_ADJUSTABLE	0x80

#define USB_DIR_OUT			0		/* to device */
#define USB_DIR_IN			0x80		/* to host */

/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_MASK			(0x03 << 5)
#define USB_TYPE_STANDARD		(0x00 << 5)
#define USB_TYPE_CLASS			(0x01 << 5)
#define USB_TYPE_VENDOR			(0x02 << 5)
#define USB_TYPE_RESERVED		(0x03 << 5)

/*
 * USB recipients, the third of three bRequestType fields
 */
#define USB_RECIP_MASK			0x1f
#define USB_RECIP_DEVICE		0x00
#define USB_RECIP_INTERFACE		0x01
#define USB_RECIP_ENDPOINT		0x02
#define USB_RECIP_OTHER			0x03
/* From Wireless USB 1.0 */
#define USB_RECIP_PORT			0x04
#define USB_RECIP_RPIPE		0x05

/*
 * Standard requests, for the bRequest field of a SETUP packet.
 *
 * These are qualified by the bRequestType field, so that for example
 * TYPE_CLASS or TYPE_VENDOR specific feature flags could be retrieved
 * by a GET_STATUS request.
 */
#define USB_REQ_GET_STATUS		0x00
#define USB_REQ_CLEAR_FEATURE		0x01
#define USB_REQ_SET_FEATURE		0x03
#define USB_REQ_SET_ADDRESS		0x05
#define USB_REQ_GET_DESCRIPTOR		0x06
#define USB_REQ_SET_DESCRIPTOR		0x07
#define USB_REQ_GET_CONFIGURATION	0x08
#define USB_REQ_SET_CONFIGURATION	0x09
#define USB_REQ_GET_INTERFACE		0x0A
#define USB_REQ_SET_INTERFACE		0x0B
#define USB_REQ_SYNCH_FRAME		0x0C

#define USB_REQ_SET_ENCRYPTION		0x0D	/* Wireless USB */
#define USB_REQ_GET_ENCRYPTION		0x0E
#define USB_REQ_RPIPE_ABORT		0x0E
#define USB_REQ_SET_HANDSHAKE		0x0F
#define USB_REQ_RPIPE_RESET		0x0F
#define USB_REQ_GET_HANDSHAKE		0x10
#define USB_REQ_SET_CONNECTION		0x11
#define USB_REQ_SET_SECURITY_DATA	0x12
#define USB_REQ_GET_SECURITY_DATA	0x13
#define USB_REQ_SET_WUSB_DATA		0x14
#define USB_REQ_LOOPBACK_DATA_WRITE	0x15
#define USB_REQ_LOOPBACK_DATA_READ	0x16
#define USB_REQ_SET_INTERFACE_DS	0x17

/* The Link Power Management (LPM) ECN defines USB_REQ_TEST_AND_SET command,
 * used by hubs to put ports into a new L1 suspend state, except that it
 * forgot to define its number ...
 */

/*
 * USB feature flags are written using USB_REQ_{CLEAR,SET}_FEATURE, and
 * are read as a bit array returned by USB_REQ_GET_STATUS.  (So there
 * are at most sixteen features of each type.)  Hubs may also support a
 * new USB_REQ_TEST_AND_SET_FEATURE to put ports into L1 suspend.
 */
#define USB_DEVICE_SELF_POWERED		0	/* (read only) */
#define USB_DEVICE_REMOTE_WAKEUP	1	/* dev may initiate wakeup */
#define USB_DEVICE_TEST_MODE		2	/* (wired high speed only) */
#define USB_DEVICE_BATTERY		2	/* (wireless) */
#define USB_DEVICE_B_HNP_ENABLE		3	/* (otg) dev may initiate HNP */
#define USB_DEVICE_WUSB_DEVICE		3	/* (wireless)*/
#define USB_DEVICE_A_HNP_SUPPORT	4	/* (otg) RH port supports HNP */
#define USB_DEVICE_A_ALT_HNP_SUPPORT	5	/* (otg) other RH port does */
#define USB_DEVICE_DEBUG_MODE		6	/* (special devices only) */

/*
 * Test Mode Selectors
 * See USB 2.0 spec Table 9-7
 */
#define	TEST_J		1
#define	TEST_K		2
#define	TEST_SE0_NAK	3
#define	TEST_PACKET	4
#define	TEST_FORCE_EN	5

/*
 * New Feature Selectors as added by USB 3.0
 * See USB 3.0 spec Table 9-6
 */
#define USB_DEVICE_U1_ENABLE	48	/* dev may initiate U1 transition */
#define USB_DEVICE_U2_ENABLE	49	/* dev may initiate U2 transition */
#define USB_DEVICE_LTM_ENABLE	50	/* dev may send LTM */
#define USB_INTRF_FUNC_SUSPEND	0	/* function suspend */

#define USB_INTR_FUNC_SUSPEND_OPT_MASK	0xFF00

#define USB_ENDPOINT_HALT		0	/* IN/OUT will STALL */

/* Bit array elements as returned by the USB_REQ_GET_STATUS request. */
#define USB_DEV_STAT_U1_ENABLED		2	/* transition into U1 state */
#define USB_DEV_STAT_U2_ENABLED		3	/* transition into U2 state */
#define USB_DEV_STAT_LTM_ENABLED	4	/* Latency tolerance messages */

/*-------------------------------------------------------------------------*/

/*
 * STANDARD DESCRIPTORS ... as returned by GET_DESCRIPTOR, or
 * (rarely) accepted by SET_DESCRIPTOR.
 *
 * Note that all multi-byte values here are encoded in little endian
 * byte order "on the wire".  Within the kernel and when exposed
 * through the Linux-USB APIs, they are not converted to cpu byte
 * order; it is the responsibility of the client code to do this.
 * The single exception is when device and configuration descriptors (but
 * not other descriptors) are read from usbfs (i.e. /proc/bus/usb/BBB/DDD);
 * in this case the fields are converted to host endianness by the kernel.
 */

/*
 * Descriptor types ... USB 2.0 spec table 9.5
 */
#define USB_DT_DEVICE			0x01
#define USB_DT_CONFIG			0x02
#define USB_DT_STRING			0x03
#define USB_DT_INTERFACE		0x04
#define USB_DT_ENDPOINT			0x05
#define USB_DT_DEVICE_QUALIFIER		0x06
#define USB_DT_OTHER_SPEED_CONFIG	0x07
#define USB_DT_INTERFACE_POWER		0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG			0x09
#define USB_DT_DEBUG			0x0a
#define USB_DT_INTERFACE_ASSOCIATION	0x0b
/* these are from the Wireless USB spec */
#define USB_DT_SECURITY			0x0c
#define USB_DT_KEY			0x0d
#define USB_DT_ENCRYPTION_TYPE		0x0e
#define USB_DT_BOS			0x0f
#define USB_DT_DEVICE_CAPABILITY	0x10
#define USB_DT_WIRELESS_ENDPOINT_COMP	0x11
#define USB_DT_WIRE_ADAPTER		0x21
#define USB_DT_RPIPE			0x22
#define USB_DT_CS_RADIO_CONTROL		0x23
/* From the T10 UAS specification */
#define USB_DT_PIPE_USAGE		0x24
/* From the USB 3.0 spec */
#define	USB_DT_SS_ENDPOINT_COMP		0x30

/* Conventional codes for class-specific descriptors.  The convention is
 * defined in the USB "Common Class" Spec (3.11).  Individual class specs
 * are authoritative for their usage, not the "common class" writeup.
 */
#define USB_DT_CS_DEVICE		(USB_TYPE_CLASS | USB_DT_DEVICE)
#define USB_DT_CS_CONFIG		(USB_TYPE_CLASS | USB_DT_CONFIG)
#define USB_DT_CS_STRING		(USB_TYPE_CLASS | USB_DT_STRING)
#define USB_DT_CS_INTERFACE		(USB_TYPE_CLASS | USB_DT_INTERFACE)
#define USB_DT_CS_ENDPOINT		(USB_TYPE_CLASS | USB_DT_ENDPOINT)

/* All standard descriptors have these 2 fields at the beginning */
struct usb_descriptor_header {
	__u8  bLength;
	__u8  bDescriptorType;
} __packed;


/*-------------------------------------------------------------------------*/

/* USB_DT_DEVICE: Device descriptor */
struct usb_device_descriptor {
	__u8  bLength;
	__u8  bDescriptorType;

	__le16 bcdUSB;
	__u8  bDeviceClass;
	__u8  bDeviceSubClass;
	__u8  bDeviceProtocol;
	__u8  bMaxPacketSize0;
	__le16 idVendor;
	__le16 idProduct;
	__le16 bcdDevice;
	__u8  iManufacturer;
	__u8  iProduct;
	__u8  iSerialNumber;
	__u8  bNumConfigurations;
} __packed;

#define USB_DT_DEVICE_SIZE		18


/*
 * Device and/or Interface Class codes
 * as found in bDeviceClass or bInterfaceClass
 * and defined by www.usb.org documents
 */
#define USB_CLASS_PER_INTERFACE		0	/* for DeviceClass */
#define USB_CLASS_AUDIO			1
#define USB_CLASS_COMM			2
#define USB_CLASS_HID			3
#define USB_CLASS_PHYSICAL		5
#define USB_CLASS_STILL_IMAGE		6
#define USB_CLASS_PRINTER		7
#define USB_CLASS_MASS_STORAGE		8
#define USB_CLASS_HUB			9
#define USB_CLASS_CDC_DATA		0x0a
#define USB_CLASS_CSCID			0x0b	/* chip+ smart card */
#define USB_CLASS_CONTENT_SEC		0x0d	/* content security */
#define USB_CLASS_VIDEO			0x0e
#define USB_CLASS_WIRELESS_CONTROLLER	0xe0
#define USB_CLASS_MISC			0xef
#define USB_CLASS_APP_SPEC		0xfe
#define USB_CLASS_VENDOR_SPEC		0xff

#define USB_SUBCLASS_VENDOR_SPEC	0xff

struct usb_ctrlrequest {
	__u8 bRequestType;
	__u8 bRequest;
	__le16 wValue;
	__le16 wIndex;
	__le16 wLength;
} __packed;

struct ethhdr {
	unsigned char	h_dest[ETH_ALEN];	/* destination eth addr	*/
	unsigned char	h_source[ETH_ALEN];	/* source ether addr	*/
	__be16		h_proto;		/* packet type ID field	*/
} __packed;

struct eth803hdr {
	u8 dest_addr[6];
	u8 src_addr[6];
	u16 h803_len;
} __packed;

struct rfc1042hdr {
	u8 llc_dsap;
	u8 llc_ssap;
	u8 llc_ctrl;
	u8 snap_oui[3];
	u16 snap_type;
} __packed;

struct rxpackethdr {
	struct eth803hdr eth803_hdr;
	struct rfc1042hdr rfc1042_hdr;
} __packed;

struct iw_point
{
	PVOID		pointer;
	USHORT		length;
	USHORT		flags;
};
	
union iwreq_data
{
	struct iw_point data;
};

struct iwreq {
	union   iwreq_data      u;
};

struct	iw_priv_args
{
	__u32		cmd;		/* Number of the ioctl to issue */
	__u16		set_args;	/* Type and number of args */
	__u16		get_args;	/* Type and number of args */
	char		name[IFNAMSIZ];	/* Name of the extension */
};


#endif
