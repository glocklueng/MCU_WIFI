#ifndef _LINUX_NETDEVICE_H
#define _LINUX_NETDEVICE_H

#include "cfg80211.h"
#include "rt_user.h"

/* source back-compat hooks */
#define SET_ETHTOOL_OPS(netdev,ops) \
	( (netdev)->ethtool_ops = (ops) )

#define HAVE_ALLOC_NETDEV		/* feature macro: alloc_xxxdev
					   functions are available. */
#define HAVE_FREE_NETDEV		/* free_netdev() */
#define HAVE_NETDEV_PRIV		/* netdev_priv() */

/* hardware address assignment types */
#define NET_ADDR_PERM		0	/* address is permanent (default) */
#define NET_ADDR_RANDOM		1	/* address is generated randomly */
#define NET_ADDR_STOLEN		2	/* address is stolen from other device */

/* Backlog congestion levels */
#define NET_RX_SUCCESS		0	/* keep 'em coming, baby */
#define NET_RX_DROP		1	/* packet dropped */

/* qdisc ->enqueue() return codes. */
#define NET_XMIT_SUCCESS	0x00
#define NET_XMIT_DROP		0x01	/* skb dropped			*/
#define NET_XMIT_CN		0x02	/* congestion notification	*/
#define NET_XMIT_POLICED	0x03	/* skb is shot by police	*/
#define NET_XMIT_MASK		0x0f	/* qdisc flags in net/sch_generic.h */

/* NET_XMIT_CN is special. It does not guarantee that this packet is lost. It
 * indicates that the device will soon be dropping packets, or already drops
 * some packets of the same priority; prompting us to send less aggressively. */
#define net_xmit_eval(e)	((e) == NET_XMIT_CN ? 0 : (e))
#define net_xmit_errno(e)	((e) != NET_XMIT_CN ? -ENOBUFS : 0)

/* Driver transmit return codes */
#define NETDEV_TX_MASK		0xf0

enum netdev_tx {
	__NETDEV_TX_MIN	 = -1,	/* make sure enum is signed */
	NETDEV_TX_OK	 = 0x00,	/* driver took care of packet */
	NETDEV_TX_BUSY	 = 0x10,	/* driver tx path was busy*/
	NETDEV_TX_LOCKED = 0x20,	/* driver tx lock was already taken */
};
typedef enum netdev_tx netdev_tx_t;


#define MAX_ADDR_LEN	32		/* Largest hardware address length */

/* Initial net device group. All devices belong to group 0 by default. */
#define INIT_NETDEV_GROUP	0


#define TC_MAX_QUEUE	16
#define TC_BITMASK	15
/* HW offloaded queuing disciplines txq count and offset maps */

struct netdev_tc_txq {
	u16 count;
	u16 offset;
};	

		
struct net_device_stats {
	unsigned long	rx_packets;
	unsigned long	tx_packets;
	unsigned long	rx_bytes;
	unsigned long	tx_bytes;
	unsigned long	rx_errors;
	unsigned long	tx_errors;
	unsigned long	rx_dropped;
	unsigned long	tx_dropped;
	unsigned long	multicast;
	unsigned long	collisions;
	unsigned long	rx_length_errors;
	unsigned long	rx_over_errors;
	unsigned long	rx_crc_errors;
	unsigned long	rx_frame_errors;
	unsigned long	rx_fifo_errors;
	unsigned long	rx_missed_errors;
	unsigned long	tx_aborted_errors;
	unsigned long	tx_carrier_errors;
	unsigned long	tx_fifo_errors;
	unsigned long	tx_heartbeat_errors;
	unsigned long	tx_window_errors;
	unsigned long	rx_compressed;
	unsigned long	tx_compressed;
};

	

struct net_device {

	/*
	 * This is the first field of the "visible" part of this structure
	 * (i.e. as seen by users in the "Space.c" file).  It is the name
	 * of the interface.
	 */
	char			name[IFNAMSIZ];

//	struct pm_qos_request_list pm_qos_req;

	/* device name hash chain */
//	struct hlist_node	name_hlist;
	/* snmp alias */
	char 			*ifalias;

	/*
	 *	I/O specific fields
	 *	FIXME: Merge these and struct ifmap into one
	 */
	unsigned long		mem_end;	/* shared mem end	*/
	unsigned long		mem_start;	/* shared mem start	*/
	unsigned long		base_addr;	/* device I/O address	*/
	unsigned int		irq;		/* device IRQ number	*/

	/*
	 *	Some hardware also needs these fields, but they are not
	 *	part of the usual set specified in Space.c.
	 */

	unsigned long		state;

//	struct list_head	dev_list;
//	struct list_head	napi_list;
//	struct list_head	unreg_list;

	/* currently active device features */
	u32			features;
	/* user-changeable features */
	u32			hw_features;
	/* user-requested features */
	u32			wanted_features;
	/* mask of features inheritable by VLAN devices */
	u32			vlan_features;

	/* Net device feature bits; if you change something,
	 * also update netdev_features_strings[] in ethtool.c */

#define NETIF_F_SG		1	/* Scatter/gather IO. */
#define NETIF_F_IP_CSUM		2	/* Can checksum TCP/UDP over IPv4. */
#define NETIF_F_NO_CSUM		4	/* Does not require checksum. F.e. loopack. */
#define NETIF_F_HW_CSUM		8	/* Can checksum all the packets. */
#define NETIF_F_IPV6_CSUM	16	/* Can checksum TCP/UDP over IPV6 */
#define NETIF_F_HIGHDMA		32	/* Can DMA to high memory. */
#define NETIF_F_FRAGLIST	64	/* Scatter/gather IO. */
#define NETIF_F_HW_VLAN_TX	128	/* Transmit VLAN hw acceleration */
#define NETIF_F_HW_VLAN_RX	256	/* Receive VLAN hw acceleration */
#define NETIF_F_HW_VLAN_FILTER	512	/* Receive filtering on VLAN */
#define NETIF_F_VLAN_CHALLENGED	1024	/* Device cannot handle VLAN packets */
#define NETIF_F_GSO		2048	/* Enable software GSO. */
#define NETIF_F_LLTX		4096	/* LockLess TX - deprecated. Please */
					/* do not use LLTX in new drivers */
#define NETIF_F_NETNS_LOCAL	8192	/* Does not change network namespaces */
#define NETIF_F_GRO		16384	/* Generic receive offload */
#define NETIF_F_LRO		32768	/* large receive offload */

/* the GSO_MASK reserves bits 16 through 23 */
#define NETIF_F_FCOE_CRC	(1 << 24) /* FCoE CRC32 */
#define NETIF_F_SCTP_CSUM	(1 << 25) /* SCTP checksum offload */
#define NETIF_F_FCOE_MTU	(1 << 26) /* Supports max FCoE MTU, 2158 bytes*/
#define NETIF_F_NTUPLE		(1 << 27) /* N-tuple filters supported */
#define NETIF_F_RXHASH		(1 << 28) /* Receive hashing offload */
#define NETIF_F_RXCSUM		(1 << 29) /* Receive checksumming offload */
#define NETIF_F_NOCACHE_COPY	(1 << 30) /* Use no-cache copyfromuser */
#define NETIF_F_LOOPBACK	(1 << 31) /* Enable loopback */

	/* Segmentation offload features */
#define NETIF_F_GSO_SHIFT	16
#define NETIF_F_GSO_MASK	0x00ff0000
/*
#define NETIF_F_TSO		(SKB_GSO_TCPV4 << NETIF_F_GSO_SHIFT)
#define NETIF_F_UFO		(SKB_GSO_UDP << NETIF_F_GSO_SHIFT)
#define NETIF_F_GSO_ROBUST	(SKB_GSO_DODGY << NETIF_F_GSO_SHIFT)
#define NETIF_F_TSO_ECN		(SKB_GSO_TCP_ECN << NETIF_F_GSO_SHIFT)
#define NETIF_F_TSO6		(SKB_GSO_TCPV6 << NETIF_F_GSO_SHIFT)
#define NETIF_F_FSO		(SKB_GSO_FCOE << NETIF_F_GSO_SHIFT)
*/
	/* Features valid for ethtool to change */
	/* = all defined minus driver/device-class-related */
#define NETIF_F_NEVER_CHANGE	(NETIF_F_VLAN_CHALLENGED | \
				  NETIF_F_LLTX | NETIF_F_NETNS_LOCAL)
#define NETIF_F_ETHTOOL_BITS	(0xff3fffff & ~NETIF_F_NEVER_CHANGE)

	/* List of features with software fallbacks. */
#define NETIF_F_GSO_SOFTWARE	(NETIF_F_TSO | NETIF_F_TSO_ECN | \
				 NETIF_F_TSO6 | NETIF_F_UFO)


#define NETIF_F_GEN_CSUM	(NETIF_F_NO_CSUM | NETIF_F_HW_CSUM)
#define NETIF_F_V4_CSUM		(NETIF_F_GEN_CSUM | NETIF_F_IP_CSUM)
#define NETIF_F_V6_CSUM		(NETIF_F_GEN_CSUM | NETIF_F_IPV6_CSUM)
#define NETIF_F_ALL_CSUM	(NETIF_F_V4_CSUM | NETIF_F_V6_CSUM)

#define NETIF_F_ALL_TSO 	(NETIF_F_TSO | NETIF_F_TSO6 | NETIF_F_TSO_ECN)

#define NETIF_F_ALL_FCOE	(NETIF_F_FCOE_CRC | NETIF_F_FCOE_MTU | \
				 NETIF_F_FSO)

	/*
	 * If one device supports one of these features, then enable them
	 * for all in netdev_increment_features.
	 */
#define NETIF_F_ONE_FOR_ALL	(NETIF_F_GSO_SOFTWARE | NETIF_F_GSO_ROBUST | \
				 NETIF_F_SG | NETIF_F_HIGHDMA |		\
				 NETIF_F_FRAGLIST | NETIF_F_VLAN_CHALLENGED)
	/*
	 * If one device doesn't support one of these features, then disable it
	 * for all in netdev_increment_features.
	 */
#define NETIF_F_ALL_FOR_ALL	(NETIF_F_NOCACHE_COPY | NETIF_F_FSO)

	/* changeable features with no special hardware requirements */
#define NETIF_F_SOFT_FEATURES	(NETIF_F_GSO | NETIF_F_GRO)

	/* Interface index. Unique device identifier	*/
	int			ifindex;
	int			iflink;

	struct net_device_stats	stats;
//	atomic_long_t		rx_dropped; 
/* dropped packets by core network
					     * Do not use this in drivers.
					     */

#ifdef CONFIG_WIRELESS_EXT
	/* List of functions to handle Wireless Extensions (instead of ioctl).
	 * See <net/iw_handler.h> for details. Jean II */
	const struct iw_handler_def *	wireless_handlers;
	/* Instance data managed by the core of Wireless Extensions. */
//	struct iw_public_data *	wireless_data;
#endif
	/* Management operations */
//	const struct net_device_ops *netdev_ops;
	const struct ethtool_ops *ethtool_ops;

	/* Hardware header description */
	const struct header_ops *header_ops;

	unsigned int		flags;	/* interface flags (a la BSD)	*/
	unsigned int		priv_flags; /* Like 'flags' but invisible to userspace. */
	unsigned short		gflags;
	unsigned short		padded;	/* How much padding added by alloc_netdev() */

	unsigned char		operstate; /* RFC2863 operstate */
	unsigned char		link_mode; /* mapping policy to operstate */

	unsigned char		if_port;	/* Selectable AUI, TP,..*/
	unsigned char		dma;		/* DMA channel		*/

	unsigned int		mtu;	/* interface MTU value		*/
	unsigned short		type;	/* interface hardware type	*/
	unsigned short		hard_header_len;	/* hardware hdr length	*/

	/* extra head- and tailroom the hardware may need, but not in all cases
	 * can this be guaranteed, especially tailroom. Some cases also use
	 * LL_MAX_HEADER instead to allocate the skb.
	 */
	unsigned short		needed_headroom;
	unsigned short		needed_tailroom;

	/* Interface address info. */
	unsigned char		perm_addr[MAX_ADDR_LEN]; /* permanent hw address */
	unsigned char		addr_assign_type; /* hw address assignment type */
	unsigned char		addr_len;	/* hardware address length	*/
	unsigned short          dev_id;		/* for shared network cards */

//	spinlock_t		addr_list_lock;
//	struct netdev_hw_addr_list	uc;	/* Unicast mac addresses */
	unsigned char	mc_addr[ETH_ALEN];	/* Multicast mac addresses */
	int			uc_promisc;
	unsigned int		promiscuity;
	unsigned int		allmulti;


	/* Protocol specific pointers */
#if 0
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	struct vlan_group __rcu	*vlgrp;		/* VLAN group */
#endif
#ifdef CONFIG_NET_DSA
	void			*dsa_ptr;	/* dsa specific data */
#endif
#endif
	void 			*atalk_ptr;	/* AppleTalk link 	*/
//	struct in_device __rcu	*ip_ptr;	/* IPv4 specific data	*/
//	struct dn_dev __rcu     *dn_ptr;        /* DECnet specific data */
//	struct inet6_dev __rcu	*ip6_ptr;       /* IPv6 specific data */
	void			*ec_ptr;	/* Econet specific data	*/
	void			*ax25_ptr;	/* AX.25 specific data */
	struct wireless_dev	*ieee80211_ptr;	/* IEEE 802.11 specific data,
						   assign before registering */

/*
 * Cache lines mostly used on receive path (including eth_type_trans())
 */
	unsigned long		last_rx;	/* Time of last Rx
						 * This should not be set in
						 * drivers, unless really needed,
						 * because network stack (bonding)
						 * use it if/when necessary, to
						 * avoid dirtying this cache line.
						 */

	struct net_device	*master; /* Pointer to master device of a group,
					  * which this device is member of.
					  */

	/* Interface address info used in eth_type_trans() */
	unsigned char		dev_addr[ETH_ALEN];	/* hw address, (before bcast
						   because most packets are
						   unicast) */

//	struct netdev_hw_addr_list	dev_addrs; 
/* list of device
						      hw addresses */

	unsigned char		broadcast[MAX_ADDR_LEN];	/* hw bcast add	*/

#ifdef CONFIG_RPS
//	struct kset		*queues_kset;

	struct netdev_rx_queue	*_rx;

	/* Number of RX queues allocated at register_netdev() time */
	unsigned int		num_rx_queues;

	/* Number of RX queues currently active in device */
	unsigned int		real_num_rx_queues;

#ifdef CONFIG_RFS_ACCEL
	/* CPU reverse-mapping for RX completion interrupts, indexed
	 * by RX queue number.  Assigned by driver.  This must only be
	 * set if the ndo_rx_flow_steer operation is defined. */
//	struct cpu_rmap		*rx_cpu_rmap;
#endif
#endif

//	rx_handler_func_t __rcu	*rx_handler;
//	void __rcu		*rx_handler_data;

//	struct netdev_queue __rcu *ingress_queue;

/*
 * Cache lines mostly used on transmit path
 */
//	struct netdev_queue	*_tx ____cacheline_aligned_in_smp;

	/* Number of TX queues allocated at alloc_netdev_mq() time  */
	unsigned int		num_tx_queues;

	/* Number of TX queues currently active in device  */
	unsigned int		real_num_tx_queues;

	/* root qdisc from userspace point of view */
//	struct Qdisc		*qdisc;

	unsigned long		tx_queue_len;	/* Max frames per queue allowed */
//	spinlock_t		tx_global_lock;

#ifdef CONFIG_XPS
//	struct xps_dev_maps __rcu *xps_maps;
#endif

	/* These may be needed for future network-power-down code. */

	/*
	 * trans_start here is expensive for high speed devices on SMP,
	 * please use netdev_queue->trans_start instead.
	 */
	unsigned long		trans_start;	/* Time (in jiffies) of last Tx	*/

	int			watchdog_timeo; /* used by dev_watchdog() */
//	struct timer_list	watchdog_timer;

	/* Number of references to this device */
//	int __percpu		*pcpu_refcnt;

	/* delayed register/unregister */
//	struct list_head	todo_list;
	/* device index hash chain */
//	struct hlist_node	index_hlist;
//
//	struct list_head	link_watch_list;

	/* register/unregister state machine */
	enum { NETREG_UNINITIALIZED=0,
	       NETREG_REGISTERED,	/* completed register_netdevice */
	       NETREG_UNREGISTERING,	/* called unregister_netdevice */
	       NETREG_UNREGISTERED,	/* completed unregister todo */
	       NETREG_RELEASED,		/* called free_netdev */
	       NETREG_DUMMY,		/* dummy device for NAPI poll */
	} reg_state:8;

	bool dismantle; /* device is going do be freed */

	enum {
		RTNL_LINK_INITIALIZED,
		RTNL_LINK_INITIALIZING,
	} rtnl_link_state:16;

	/* Called from unregister, can be used to call free_netdev */
	void (*destructor)(struct net_device *dev);

#ifdef CONFIG_NETPOLL
//	struct netpoll_info	*npinfo;
#endif

#ifdef CONFIG_NET_NS
	/* Network namespace this network device is inside */
//	struct net		*nd_net;
#endif
	void				*ml_priv;
#if 0
	/* mid-layer private */
	union {
		
		struct pcpu_lstats __percpu	*lstats; /* loopback stats */
		struct pcpu_tstats __percpu	*tstats; /* tunnel stats */
		struct pcpu_dstats __percpu	*dstats; /* dummy stats */
	};
	/* GARP */
	struct garp_port __rcu	*garp_port;

	/* class/net/name entry */
	struct device		dev;
	/* space for optional device, statistics, and wireless sysfs groups */
	const struct attribute_group *sysfs_groups[4];
#endif
	/* rtnetlink link ops */
	const struct rtnl_link_ops *rtnl_link_ops;

	/* for setting kernel sock attribute on TCP connection setup */
#define GSO_MAX_SIZE		65536
	unsigned int		gso_max_size;

#ifdef CONFIG_DCB
	/* Data Center Bridging netlink ops */
//	const struct dcbnl_rtnl_ops *dcbnl_ops;
#endif
	u8 num_tc;
	struct netdev_tc_txq tc_to_txq[TC_MAX_QUEUE];
	u8 prio_tc_map[TC_BITMASK + 1];

#if defined(CONFIG_FCOE) || defined(CONFIG_FCOE_MODULE)
	/* max exchange id for FCoE LRO by ddp */
	unsigned int		fcoe_ddp_xid;
#endif
	/* n-tuple filter list attached to this device */
//	struct ethtool_rx_ntuple_list ethtool_ntuple_list;

	/* phy device may attach itself for hardware timestamping */
//	struct phy_device *phydev;

	/* group the device belongs to */
	int group;
};
#define to_net_dev(d) container_of(d, struct net_device, dev)

#define	NETDEV_ALIGN		32

#endif
