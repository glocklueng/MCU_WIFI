/*
 *	Definitions for the 'struct sk_buff' memory handlers.
 *
 *	Authors:
 *		Alan Cox, <gw4pts@gw4pts.ampr.org>
 *		Florian La Roche, <rzsfl@rz.uni-sb.de>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#ifndef _LINUX_SKBUFF_H
#define _LINUX_SKBUFF_H
/*
#include <linux/kernel.h>
#include <linux/kmemcheck.h>
#include <linux/compiler.h>
#include <linux/time.h>
#include <linux/cache.h>

#include <asm/atomic.h>
#include <asm/types.h>
#include <linux/spinlock.h>
#include <linux/net.h>
#include <linux/textsearch.h>
#include <net/checksum.h>
#include <linux/rcupdate.h>
#include <linux/dmaengine.h>
#include <linux/hrtimer.h>*/

#define BITS_PER_LONG 32

/* Don't change this without changing skb_csum_unnecessary! */
#define CHECKSUM_NONE 0
#define CHECKSUM_UNNECESSARY 1
#define CHECKSUM_COMPLETE 2
#define CHECKSUM_PARTIAL 3

# define NSEC_PER_SEC			1000000000ULL

#define ALIGN(x, a)		(((x) + (a - 1)) & \
				 ~(a - 1))

#define SKB_DATA_ALIGN(X)	(((X) + (4 - 1)) & \
				 ~(4 - 1))
#define SKB_WITH_OVERHEAD(X)	\
	((X) - SKB_DATA_ALIGN(sizeof(struct skb_shared_info)))
#define SKB_MAX_ORDER(X, ORDER) \
	SKB_WITH_OVERHEAD((PAGE_SIZE << (ORDER)) - (X))
#define SKB_MAX_HEAD(X)		(SKB_MAX_ORDER((X), 0))
#define SKB_MAX_ALLOC		(SKB_MAX_ORDER(0, 2))



struct page {
	unsigned long flags;		/* Atomic flags, some possibly*/
	atomic_t _count;		/* Usage count, see below. */
	char  *addr;
};

#if 0
struct ip_options {
	__u32		faddr;			/* Saved first hop address */
	unsigned char	optlen;
	unsigned char srr;
	unsigned char rr;
	unsigned char ts;
	unsigned char is_setbyuser:1,	/* Set by setsockopt?		      */
		      is_data:1,	/* Options in __data, rather than skb */
		      is_strictroute:1, /* Strict source route		      */
		      srr_is_hit:1,	/* Packet destination addr was our one*/
		      is_changed:1,	/* IP checksum more not valid	      */
		      rr_needaddr:1,	/* Need to record addr of outgoing dev*/
		      ts_needtime:1,	/* Need to record timestamp	      */
		      ts_needaddr:1;	/* Need to record addr of outgoing dev*/
	unsigned char router_alert;
	unsigned char __pad1;
	unsigned char __pad2;
	unsigned char __data[1];//mark
};
#endif

#define IPSKB_FORWARDED		1
#define IPSKB_XFRM_TUNNEL_SIZE	2
#define IPSKB_XFRM_TRANSFORMED	4
#define IPSKB_FRAG_COMPLETE	8
#define IPSKB_REROUTED		16

#define SO_EE_ORIGIN_NONE	0
#define SO_EE_ORIGIN_LOCAL	1
#define SO_EE_ORIGIN_ICMP	2
#define SO_EE_ORIGIN_ICMP6	3
#define SO_EE_ORIGIN_TIMESTAMPING 4
/*
struct inet_skb_parm {
	struct ip_options	opt;		
	unsigned char		flags;
};*/

struct scatterlist {
	unsigned long	page_link;
	unsigned int	offset;
	unsigned int	length;
	dma_addr_t	dma_address;
};

#define sg_dma_address(sg)	((sg)->dma_address)

#ifdef CONFIG_NEED_SG_DMA_LENGTH
#define sg_dma_len(sg)		((sg)->dma_length)
#else
#define sg_dma_len(sg)		((sg)->length)
#endif

/* Sock flags */
enum sock_flags {
	SOCK_DEAD,
	SOCK_DONE,
	SOCK_URGINLINE,
	SOCK_KEEPOPEN,
	SOCK_LINGER,
	SOCK_DESTROY,
	SOCK_BROADCAST,
	SOCK_TIMESTAMP,
	SOCK_ZAPPED,
	SOCK_USE_WRITE_QUEUE, /* whether to call sk->sk_write_space in sock_wfree */
	SOCK_DBG, /* %SO_DEBUG setting */
	SOCK_RCVTSTAMP, /* %SO_TIMESTAMP setting */
	SOCK_RCVTSTAMPNS, /* %SO_TIMESTAMPNS setting */
	SOCK_LOCALROUTE, /* route locally only, %SO_DONTROUTE setting */
	SOCK_QUEUE_SHRUNK, /* write queue has been shrunk recently */
	SOCK_TIMESTAMPING_TX_HARDWARE,  /* %SOF_TIMESTAMPING_TX_HARDWARE */
	SOCK_TIMESTAMPING_TX_SOFTWARE,  /* %SOF_TIMESTAMPING_TX_SOFTWARE */
	SOCK_TIMESTAMPING_RX_HARDWARE,  /* %SOF_TIMESTAMPING_RX_HARDWARE */
	SOCK_TIMESTAMPING_RX_SOFTWARE,  /* %SOF_TIMESTAMPING_RX_SOFTWARE */
	SOCK_TIMESTAMPING_SOFTWARE,     /* %SOF_TIMESTAMPING_SOFTWARE */
	SOCK_TIMESTAMPING_RAW_HARDWARE, /* %SOF_TIMESTAMPING_RAW_HARDWARE */
	SOCK_TIMESTAMPING_SYS_HARDWARE, /* %SOF_TIMESTAMPING_SYS_HARDWARE */
	SOCK_FASYNC, /* fasync() active */
	SOCK_RXQ_OVFL,
};

struct sock_extended_err {
	__u32	ee_errno;	
	__u8	ee_origin;
	__u8	ee_type;
	__u8	ee_code;
	__u8	ee_pad;
	__u32   ee_info;
	__u32   ee_data;
};
struct sock_exterr_skb {
/*	union {
		struct inet_skb_parm	h4;
		
	} header;*/
	struct sock_extended_err	ee;
	u16				addr_offset;
	__be16				port;
};

#define SKB_EXT_ERR(skb) ((struct sock_exterr_skb *) ((skb)->cb))

#define atomic_inc(v)		atomic_add(1, v)
#define atomic_dec(v)		atomic_sub(1, v)

#define atomic_inc_and_test(v)	(atomic_add_return(1, v) == 0)
#define atomic_dec_and_test(v)	(atomic_sub_return(1, v) == 0)
#define atomic_inc_return(v)    (atomic_add_return(1, v))
#define atomic_dec_return(v)    (atomic_sub_return(1, v))
#define atomic_sub_and_test(i, v) (atomic_sub_return(i, v) == 0)


/* A. Checksumming of received packets by device.
 *
 *	NONE: device failed to checksum this packet.
 *		skb->csum is undefined.
 *
 *	UNNECESSARY: device parsed packet and wouldbe verified checksum.
 *		skb->csum is undefined.
 *	      It is bad option, but, unfortunately, many of vendors do this.
 *	      Apparently with secret goal to sell you new device, when you
 *	      will add new protocol to your host. F.e. IPv6. 8)
 *
 *	COMPLETE: the most generic way. Device supplied checksum of _all_
 *	    the packet as seen by netif_rx in skb->csum.
 *	    NOTE: Even if device supports only some protocols, but
 *	    is able to produce some skb->csum, it MUST use COMPLETE,
 *	    not UNNECESSARY.
 *
 *	PARTIAL: identical to the case for output below.  This may occur
 *	    on a packet received directly from another Linux OS, e.g.,
 *	    a virtualised Linux kernel on the same host.  The packet can
 *	    be treated in the same way as UNNECESSARY except that on
 *	    output (i.e., forwarding) the checksum must be filled in
 *	    by the OS or the hardware.
 *
 * B. Checksumming on output.
 *
 *	NONE: skb is checksummed by protocol or csum is not required.
 *
 *	PARTIAL: device is required to csum packet as seen by hard_start_xmit
 *	from skb->csum_start to the end and to record the checksum
 *	at skb->csum_start + skb->csum_offset.
 *
 *	Device must show its capabilities in dev->features, set
 *	at device setup time.
 *	NETIF_F_HW_CSUM	- it is clever device, it is able to checksum
 *			  everything.
 *	NETIF_F_NO_CSUM - loopback or reliable single hop media.
 *	NETIF_F_IP_CSUM - device is dumb. It is able to csum only
 *			  TCP/UDP over IPv4. Sigh. Vendors like this
 *			  way by an unknown reason. Though, see comment above
 *			  about CHECKSUM_UNNECESSARY. 8)
 *	NETIF_F_IPV6_CSUM about as dumb as the last one but does IPv6 instead.
 *
 *	Any questions? No questions, good. 		--ANK
 */
/*
struct net_device;
struct scatterlist;
struct pipe_inode_info;

#if defined(CONFIG_NF_CONNTRACK) || defined(CONFIG_NF_CONNTRACK_MODULE)
struct nf_conntrack {
	atomic_t use;
};
#endif

#ifdef CONFIG_BRIDGE_NETFILTER
struct nf_bridge_info {
	atomic_t use;
	struct net_device *physindev;
	struct net_device *physoutdev;
	unsigned int mask;
	unsigned long data[32 / sizeof(unsigned long)];
};
#endif
*/
struct sk_buff_head {
	/* These two members must be first. */
	struct sk_buff	*next;
	struct sk_buff	*prev;

	__u32		qlen;
	spinlock_t	lock;
};

struct sk_buff;

/* To allow 64K frame to be packed as single skb without frag_list. Since
 * GRO uses frags we allocate at least 16 regardless of page size.
 */
//#if (65536/PAGE_SIZE + 2) < 16
#define MAX_SKB_FRAGS 4UL		//16
//#else
//#define MAX_SKB_FRAGS (65536/PAGE_SIZE + 2)
//
#endif

typedef struct skb_frag_struct skb_frag_t;

struct skb_frag_struct {
	struct page *page;
#if (BITS_PER_LONG > 32) || (PAGE_SIZE >= 65536)
	__u32 page_offset;
	__u32 size;
#else
	__u16 page_offset;
	__u16 size;
#endif
};


#define HAVE_HW_TIME_STAMP

/**
 * struct skb_shared_hwtstamps - hardware time stamps
 * @hwtstamp:	hardware time stamp transformed into duration
 *		since arbitrary point in time
 * @syststamp:	hwtstamp transformed to system time base
 *
 * Software time stamps generated by ktime_get_real() are stored in
 * skb->tstamp. The relation between the different kinds of time
 * stamps is as follows:
 *
 * syststamp and tstamp can be compared against each other in
 * arbitrary combinations.  The accuracy of a
 * syststamp/tstamp/"syststamp from other device" comparison is
 * limited by the accuracy of the transformation into system time
 * base. This depends on the device driver and its underlying
 * hardware.
 *
 * hwtstamps can only be compared against other hwtstamps from
 * the same device.
 *
 * This structure is attached to packets as part of the
 * &skb_shared_info. Use skb_hwtstamps() to get a pointer.
 */
union ktime {
	s32	tv64;
/*	struct {
	s32	nsec, sec;
	} tv;*/
};

typedef union ktime ktime_t;	

struct skb_shared_hwtstamps {
	ktime_t	hwtstamp;
	ktime_t	syststamp;
};

/* Definitions for tx_flags in struct skb_shared_info */
enum {
	/* generate hardware time stamp */
	SKBTX_HW_TSTAMP = 1 << 0,

	/* generate software time stamp */
	SKBTX_SW_TSTAMP = 1 << 1,

	/* device driver is going to provide hardware time stamp */
	SKBTX_IN_PROGRESS = 1 << 2,

	/* ensure the originating sk reference is available on driver level */
	SKBTX_DRV_NEEDS_SK_REF = 1 << 3,
};

/* This data is invariant across clones and lives at
 * the end of the header data, ie. at skb->end.
 */
struct skb_shared_info {
	unsigned short	nr_frags;
	unsigned short	gso_size;
	/* Warning: this field is not always filled in (UFO)! */
	unsigned short	gso_segs;
	unsigned short  gso_type;
	__be32          ip6_frag_id;
	__u8		tx_flags;
	struct sk_buff	*frag_list;
	struct skb_shared_hwtstamps hwtstamps;

	/*
	 * Warning : all fields before dataref are cleared in __alloc_skb()
	 */
	atomic_t	dataref;

	/* Intermediate layers must ensure that destructor_arg
	 * remains valid until skb destructor */
	void *		destructor_arg;
	/* must be last field, see pskb_expand_head() */
	skb_frag_t	frags[MAX_SKB_FRAGS];
};

/* We divide dataref into two halves.  The higher 16 bits hold references
 * to the payload part of skb->data.  The lower 16 bits hold references to
 * the entire skb->data.  A clone of a headerless skb holds the length of
 * the header in skb->hdr_len.
 *
 * All users must obey the rule that the skb->data reference count must be
 * greater than or equal to the payload reference count.
 *
 * Holding a reference to the payload part means that the user does not
 * care about modifications to the header part of skb->data.
 */
#define SKB_DATAREF_SHIFT 16
#define SKB_DATAREF_MASK ((1 << SKB_DATAREF_SHIFT) - 1)


enum {
	SKB_FCLONE_UNAVAILABLE,
	SKB_FCLONE_ORIG,
	SKB_FCLONE_CLONE,
};

enum {
	SKB_GSO_TCPV4 = 1 << 0,
	SKB_GSO_UDP = 1 << 1,

	/* This indicates the skb is from an untrusted source. */
	SKB_GSO_DODGY = 1 << 2,

	/* This indicates the tcp segment has CWR set. */
	SKB_GSO_TCP_ECN = 1 << 3,

	SKB_GSO_TCPV6 = 1 << 4,

	SKB_GSO_FCOE = 1 << 5,
};

#if BITS_PER_LONG > 32
#define NET_SKBUFF_DATA_USES_OFFSET 1
#endif

#ifdef NET_SKBUFF_DATA_USES_OFFSET
typedef unsigned int sk_buff_data_t;
#else
typedef unsigned char *sk_buff_data_t;
#endif

#if defined(CONFIG_NF_DEFRAG_IPV4) || defined(CONFIG_NF_DEFRAG_IPV4_MODULE) || \
    defined(CONFIG_NF_DEFRAG_IPV6) || defined(CONFIG_NF_DEFRAG_IPV6_MODULE)
#define NET_SKBUFF_NF_DEFRAG_NEEDED 1
#endif

/** 
 *	struct sk_buff - socket buffer
 *	@next: Next buffer in list
 *	@prev: Previous buffer in list
 *	@sk: Socket we are owned by
 *	@tstamp: Time we arrived
 *	@dev: Device we arrived on/are leaving by
 *	@transport_header: Transport layer header
 *	@network_header: Network layer header
 *	@mac_header: Link layer header
 *	@_skb_refdst: destination entry (with norefcount bit)
 *	@sp: the security path, used for xfrm
 *	@cb: Control buffer. Free for use by every layer. Put private vars here
 *	@len: Length of actual data
 *	@data_len: Data length
 *	@mac_len: Length of link layer header
 *	@hdr_len: writable header length of cloned skb
 *	@csum: Checksum (must include start/offset pair)
 *	@csum_start: Offset from skb->head where checksumming should start
 *	@csum_offset: Offset from csum_start where checksum should be stored
 *	@local_df: allow local fragmentation
 *	@cloned: Head may be cloned (check refcnt to be sure)
 *	@nohdr: Payload reference only, must not modify header
 *	@pkt_type: Packet class
 *	@fclone: skbuff clone status
 *	@ip_summed: Driver fed us an IP checksum
 *	@priority: Packet queueing priority
 *	@users: User count - see {datagram,tcp}.c
 *	@protocol: Packet protocol from driver
 *	@truesize: Buffer size 
 *	@head: Head of buffer
 *	@data: Data head pointer
 *	@tail: Tail pointer
 *	@end: End pointer
 *	@destructor: Destruct function
 *	@mark: Generic packet mark
 *	@nfct: Associated connection, if any
 *	@ipvs_property: skbuff is owned by ipvs
 *	@peeked: this packet has been seen already, so stats have been
 *		done for it, don't do them again
 *	@nf_trace: netfilter packet trace flag
 *	@nfctinfo: Relationship of this skb to the connection
 *	@nfct_reasm: netfilter conntrack re-assembly pointer
 *	@nf_bridge: Saved data about a bridged frame - see br_netfilter.c
 *	@skb_iif: ifindex of device we arrived on
 *	@rxhash: the packet hash computed on receive
 *	@queue_mapping: Queue mapping for multiqueue devices
 *	@tc_index: Traffic control index
 *	@tc_verd: traffic control verdict
 *	@ndisc_nodetype: router type (from link layer)
 *	@dma_cookie: a cookie to one of several possible DMA operations
 *		done by skb DMA functions
 *	@secmark: security marking
 *	@vlan_tci: vlan tag control information
 */

struct sk_buff {
	/* These two members must be first. */
	struct sk_buff		*next;
	struct sk_buff		*prev;

	ktime_t			tstamp;

	struct sock		*sk;
	struct net_device	*dev;

	/*
	 * This is the control buffer. It is free to use for every
	 * layer. Please put your private variables there. If you
	 * want to keep them across layers you have to do a skb_clone()
	 * first. This is owned by whoever has the skb queued ATM.
	 */
	char			cb[48] /*__aligned(8)*/;

	unsigned long		_skb_refdst;

	unsigned int		len,
				data_len;
	__u16			mac_len,
				hdr_len;
	union {
		__wsum		csum;
		struct {
			__u16	csum_start;
			__u16	csum_offset;
		}cs_w;
	}cs;
	__u32			priority;

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

	__be16			protocol;

	void			(*destructor)(struct sk_buff *skb);

	int			skb_iif;

	__u32			rxhash;

	__u16			queue_mapping;

	__u8			ooo_okay:1;

	union {
		__u32		mark;
		__u32		dropcount;
	}mk;

	__u16			vlan_tci;

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

#define kmap_skb_frag(X) ((X)->page->addr)

#define kunmap_skb_frag(x)


#if 1
/*
 *	Handling routines are only of interest to the kernel
 */
//#include <linux/slab.h>

//#include <asm/system.h>

/*
 * skb might have a dst pointer attached, refcounted or not.
 * _skb_refdst low order bit is set if refcount was _not_ taken
 */
#define SKB_DST_NOREF	1UL
#define SKB_DST_PTRMASK	~(SKB_DST_NOREF)

struct dst_entry {
//	struct rcu_head		rcu_head;
	struct dst_entry	*child;
	struct net_device       *dev;
//	struct  dst_ops	        *ops;
	unsigned long		_metrics;
	unsigned long		expires;
	struct dst_entry	*path;
	struct neighbour	*neighbour;
//	struct hh_cache		*hh;

	void			*__pad1;

	int			(*input)(struct sk_buff*);
	int			(*output)(struct sk_buff*);

	short			error;
	short			obsolete;
	unsigned short		header_len;	/* more space at head required */
	unsigned short		trailer_len;	/* space to reserve at tail */

	__u32			__pad2;
	/*
	 * __refcnt wants to be on a different cache line from
	 * input/output/ops or performance tanks badly
	 */
	atomic_t		__refcnt;	/* client references	*/
	int			__use;
	unsigned long		lastuse;
	int			flags;
#define DST_HOST		0x0001
#define DST_NOXFRM		0x0002
#define DST_NOPOLICY		0x0004
#define DST_NOHASH		0x0008
#define DST_NOCACHE		0x0010
#define DST_NOCOUNT		0x0020
//	union {
		struct dst_entry	*next;
//		struct rtable __rcu	*rt_next;
//		struct rt6_info		*rt6_next;
//		struct dn_route __rcu	*dn_next;
//	};
};


struct rtable {
	struct dst_entry	dst;

	/* Lookup key. */
	__be32			rt_key_dst;
	__be32			rt_key_src;

	int			rt_genid;
	unsigned		rt_flags;
	__u16			rt_type;
	__u8			rt_key_tos;

	__be32			rt_dst;	/* Path destination	*/
	__be32			rt_src;	/* Path source		*/
	int			rt_route_iif;
	int			rt_iif;
	int			rt_oif;
	__u32			rt_mark;

	/* Info on neighbour */
	__be32			rt_gateway;

	/* Miscellaneous cached information */
	__be32			rt_spec_dst; /* RFC1122 specific destination */
	u32			rt_peer_genid;
//	struct inet_peer	*peer; /* long-living peer info */
//	struct fib_info		*fi; /* for client ref to shared metrics */
};

/**
 * skb_dst - returns skb dst_entry
 * @skb: buffer
 *
 * Returns skb dst_entry, regardless of reference taken or not.
 */
#define skb_dst(SKB) 	((struct dst_entry *)(SKB->_skb_refdst & SKB_DST_PTRMASK))
#if 0
static inline struct dst_entry *skb_dst(const struct sk_buff *skb)
{
	/* If refdst was not refcounted, check we still are in a 
	 * rcu_read_lock section
	 */
	WARN_ON((skb->_skb_refdst & SKB_DST_NOREF) &&
		!rcu_read_lock_held() &&
		!rcu_read_lock_bh_held());
	return (struct dst_entry *)(skb->_skb_refdst & SKB_DST_PTRMASK);
}
#endif
/**
 * skb_dst_set - sets skb dst
 * @skb: buffer
 * @dst: dst entry
 *
 * Sets skb dst, assuming a reference was taken on dst and should
 * be released by skb_dst_drop()
 */
#define skb_dst_set(SKB, DST)		do{		\
	SKB->_skb_refdst = (unsigned long)DST;	\
}while(0)

extern void skb_dst_set_noref(struct sk_buff *skb, struct dst_entry *dst);

/**
 * skb_dst_is_noref - Test if skb dst isn't refcounted
 * @skb: buffer
 */
#define skb_dst_is_noref(SKB)   ((SKB->_skb_refdst & SKB_DST_NOREF) && skb_dst(SKB))

#define skb_rtable(SKB)	((struct rtable *)skb_dst(SKB))

extern void kfree_skb(struct sk_buff *skb);
extern void consume_skb(struct sk_buff *skb);
extern void	       __kfree_skb(struct sk_buff *skb);

extern struct sk_buff *__alloc_skb(const char* name, unsigned int size, gfp_t gfp_mask, int fclone, int node);


#define alloc_skb(size, priority) __alloc_skb(__FUNCTION__, size, priority, 0, NUMA_NO_NODE)

#define  alloc_skb_fclone(size, priority)  ((struct sk_buff *)__alloc_skb(__FUNCTION__, size, priority, 1, NUMA_NO_NODE))


extern bool skb_recycle_check(struct sk_buff *skb, int skb_size);

extern struct sk_buff *skb_morph(struct sk_buff *dst, struct sk_buff *src);
extern struct sk_buff *skb_clone(struct sk_buff *skb,
				 gfp_t priority);
extern struct sk_buff *skb_copy(const struct sk_buff *skb,
				gfp_t priority);
extern struct sk_buff *pskb_copy(struct sk_buff *skb,
				 gfp_t gfp_mask);
extern int	       pskb_expand_head(struct sk_buff *skb,
					int nhead, int ntail,
					gfp_t gfp_mask);
extern struct sk_buff *skb_realloc_headroom(struct sk_buff *skb,
					    unsigned int headroom);
extern struct sk_buff *skb_copy_expand(const struct sk_buff *skb,
				       int newheadroom, int newtailroom,
				       gfp_t priority);
//extern int	       skb_to_sgvec(struct sk_buff *skb,
//				    struct scatterlist *sg, int offset,
//				    int len);
extern int	       skb_cow_data(struct sk_buff *skb, int tailbits,
				    struct sk_buff **trailer);
extern int	       skb_pad(struct sk_buff *skb, int pad);
#define dev_kfree_skb(a)	consume_skb(a)

extern int skb_append_datato_frags(struct sock *sk, struct sk_buff *skb,
			int getfrag(void *from, char *to, int offset,
			int len,int odd, struct sk_buff *skb),
			void *from, int length);

extern struct sk_buff *dev_alloc_skb(unsigned int length);

extern struct sk_buff *__dev_alloc_skb(unsigned int length,
					      gfp_t gfp_mask);
void skb_check(void *buff);

struct skb_seq_state {
	__u32		lower_offset;
	__u32		upper_offset;
	__u32		frag_idx;
	__u32		stepped_offset;
	struct sk_buff	*root_skb;
	struct sk_buff	*cur_skb;
	__u8		*frag_data;
};

//extern void	      skb_prepare_seq_read(struct sk_buff *skb,
//					   unsigned int from, unsigned int to,
//					   /
//					   struct skb_seq_state *st);
//extern unsigned int   skb_seq_read(unsigned int consumed, const u8 **data,
//				   struct skb_seq_state *st);
//extern void	      skb_abort_seq_read(struct skb_seq_state *st);

extern unsigned int   skb_find_text(struct sk_buff *skb, unsigned int from,
				    unsigned int to, struct ts_config *config,
				    void *state);

extern __u32 __skb_get_rxhash(struct sk_buff *skb);

extern unsigned char *skb_push(struct sk_buff *skb, unsigned int len);
/*
static inline __u32 skb_get_rxhash(struct sk_buff *skb)
{
	if (!skb->rxhash)
		skb->rxhash = __skb_get_rxhash(skb);

	return skb->rxhash;
}*/

#define skb_end_pointer(SKB) (SKB->end)


/* Internal */
#define skb_shinfo(SKB)	((struct skb_shared_info *)(skb_end_pointer(SKB)))

#define skb_hwtstamps(SKB)	(&skb_shinfo(SKB)->hwtstamps)
/**
 *	skb_queue_empty - check if a queue is empty
 *	@list: queue head
 *
 *	Returns true if the queue is empty, false otherwise.
 */
#define skb_queue_empty(LIST)	(LIST->next == (struct sk_buff *)LIST)

/**
 *	skb_queue_is_last - check if skb is the last entry in the queue
 *	@list: queue head
 *	@skb: buffer
 *
 *	Returns true if @skb is the last buffer on the list.
 */
#define skb_queue_is_last(LIST,  SKB)  (SKB->next == (struct sk_buff *)LIST)


/**
 *	skb_queue_is_first - check if skb is the first entry in the queue
 *	@list: queue head
 *	@skb: buffer
 *
 *	Returns true if @skb is the first buffer on the list.
 */
#define skb_queue_is_first(LIST, SKB)	 (SKB->prev == (struct sk_buff *)LIST)

/**
 *	skb_queue_next - return the next packet in the queue
 *	@list: queue head
 *	@skb: current buffer
 *
 *	Return the next packet in @list after @skb.  It is only valid to
 *	call this if skb_queue_is_last() evaluates to false.
 */
#define skb_queue_next(LIST, SKB) )  (SKB->next)

/**
 *	skb_queue_prev - return the prev packet in the queue
 *	@list: queue head
 *	@skb: current buffer
 *
 *	Return the prev packet in @list before @skb.  It is only valid to
 *	call this if skb_queue_is_first() evaluates to false.
 */

#define skb_queue_prev(LIST, SKB) )  (SKB->prev)

/**
 *	skb_get - reference buffer
 *	@skb: buffer to reference
 *
 *	Makes another reference to a socket buffer and returns a pointer
 *	to the buffer.
 */
#define skb_get(SKB)  (SKB)

/*
 * If users == 1, we are the only owner and are can avoid redundant
 * atomic change.
 */

/**
 *	skb_cloned - is the buffer a clone
 *	@skb: buffer to check
 *
 *	Returns true if the buffer was generated with skb_clone() and is
 *	one of multiple shared copies of the buffer. Cloned buffers are
 *	shared data so must not be written to under normal circumstances.
 */
#define skb_cloned(SKB)	(SKB->cloned &&(atomic_read(&skb_shinfo(SKB)->dataref) & SKB_DATAREF_MASK) != 1)

/**
 *	skb_header_cloned - is the header a clone
 *	@skb: buffer to check
 *
 *	Returns true if modifying the header part of the buffer requires
 *	the data to be copied.
 */


/**
 *	skb_header_release - release reference to header
 *	@skb: buffer to operate on
 *
 *	Drop a reference to the header part of the buffer.  This is done
 *	by acquiring a payload reference.  You must not read from the header
 *	part of skb->data after this.
 */


/**
 *	skb_shared - is the buffer shared
 *	@skb: buffer to check
 *
 *	Returns true if more than one person has a reference to this
 *	buffer.
 */
#define  skb_shared(SKB)	(atomic_read(&SKB->users) != 1)


/**
 *	skb_share_check - check if buffer is shared and if so clone it
 *	@skb: buffer to check
 *	@pri: priority for memory allocation
 *
 *	If the buffer is shared the buffer is cloned and the old copy
 *	drops a reference. A new clone with a single reference is returned.
 *	If the buffer is not shared the original buffer is returned. When
 *	being called from interrupt status or with spinlocks held pri must
 *	be GFP_ATOMIC.
 *
 *	NULL is returned on a memory allocation failure.
 */


/*
 *	Copy shared buffers into a new sk_buff. We effectively do COW on
 *	packets to handle cases where we have a local reader and forward
 *	and a couple of other messy ones. The normal one is tcpdumping
 *	a packet thats being forwarded.
 */

/**
 *	skb_unshare - make a copy of a shared buffer
 *	@skb: buffer to check
 *	@pri: priority for memory allocation
 *
 *	If the socket buffer is a clone then this function creates a new
 *	copy of the data, drops a reference count on the old copy and returns
 *	the new copy with the reference count at 1. If the buffer is not a clone
 *	the original buffer is returned. When called with a spinlock held or
 *	from interrupt state @pri must be %GFP_ATOMIC
 *
 *	%NULL is returned on a memory allocation failure.
 */


/**
 *	skb_peek - peek at the head of an &sk_buff_head
 *	@list_: list to peek at
 *
 *	Peek an &sk_buff. Unlike most other operations you _MUST_
 *	be careful with this one. A peek leaves the buffer on the
 *	list and someone else may run off with it. You must hold
 *	the appropriate locks or have a private queue to do this.
 *
 *	Returns %NULL for an empty list or a pointer to the head element.
 *	The reference count is not incremented and the reference is therefore
 *	volatile. Use with caution.
 */


//bool skb_partial_csum_set(struct sk_buff *skb, u16 start, u16 off);


int skb_tailroom(const struct sk_buff *skb);
void skb_reserve(struct sk_buff *skb, int len);
unsigned int skb_headroom(const struct sk_buff *skb);
void skb_reset_mac_header(struct sk_buff *skb);
unsigned char *skb_pull_inline(struct sk_buff *skb, unsigned int len);
unsigned char *skb_mac_header(const struct sk_buff *skb);


#endif	/* __KERNEL__ */

