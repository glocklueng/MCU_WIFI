#ifndef _MAC80211_DEF_H
#define _MAC80211_DEF_H

#include "sys_misc.h"
#include "cfg80211.h"

#define EXPORT_SYMBOL(x)

#define netif_carrier_off(x)
#define netif_stop_queue(x)
#define netif_start_queue(x)
#define netif_carrier_on(x)
#define netif_wake_queue(x)

#define rcu_read_lock()
#define rcu_read_unlock()

#define rcu_dereference(x) x

#define get_random_bytes os_get_random

#define kmalloc_node_track_caller(x,y,z) mem_malloc(x)

#define kmem_cache_free(x,y) mem_free(y)

#define NUMA_NO_NODE 1

#define NET_SKB_PAD	   32

#define SKB_DST_NOREF	1UL
#define SKB_DST_PTRMASK	~(SKB_DST_NOREF)

//#define kmap_skb_frag(x) (x)

//#define kunmap_skb_frag(x)

//#define SKB_DATA_ALIGN(x)  ((x + 3)&(~0x03))

#define SKB_DATAREF_SHIFT 16
#define SKB_DATAREF_MASK ((1 << SKB_DATAREF_SHIFT) - 1)

#define skb_walk_frags(skb, iter)	\
	for (iter = skb_shinfo(skb)->frag_list; iter; iter = iter->next)

#define max_t(X,Y,Z) max(Y,Z)

#if 0
enum ieee80211_radiotap_type {
	IEEE80211_RADIOTAP_TSFT = 0,
	IEEE80211_RADIOTAP_FLAGS = 1,
	IEEE80211_RADIOTAP_RATE = 2,
	IEEE80211_RADIOTAP_CHANNEL = 3,
	IEEE80211_RADIOTAP_FHSS = 4,
	IEEE80211_RADIOTAP_DBM_ANTSIGNAL = 5,
	IEEE80211_RADIOTAP_DBM_ANTNOISE = 6,
	IEEE80211_RADIOTAP_LOCK_QUALITY = 7,
	IEEE80211_RADIOTAP_TX_ATTENUATION = 8,
	IEEE80211_RADIOTAP_DB_TX_ATTENUATION = 9,
	IEEE80211_RADIOTAP_DBM_TX_POWER = 10,
	IEEE80211_RADIOTAP_ANTENNA = 11,
	IEEE80211_RADIOTAP_DB_ANTSIGNAL = 12,
	IEEE80211_RADIOTAP_DB_ANTNOISE = 13,
	IEEE80211_RADIOTAP_RX_FLAGS = 14,
	IEEE80211_RADIOTAP_TX_FLAGS = 15,
	IEEE80211_RADIOTAP_RTS_RETRIES = 16,
	IEEE80211_RADIOTAP_DATA_RETRIES = 17,

	IEEE80211_RADIOTAP_MCS = 19,

	/* valid in every it_present bitmap, even vendor namespaces */
	IEEE80211_RADIOTAP_RADIOTAP_NAMESPACE = 29,
	IEEE80211_RADIOTAP_VENDOR_NAMESPACE = 30,
	IEEE80211_RADIOTAP_EXT = 31
};
#endif



/* Wireless Identification */
#define SIOCSIWCOMMIT	0x8B00		/* Commit pending changes to driver */
#define SIOCGIWNAME	0x8B01		/* get name == wireless protocol */
/* SIOCGIWNAME is used to verify the presence of Wireless Extensions.
 * Common values : "IEEE 802.11-DS", "IEEE 802.11-FH", "IEEE 802.11b"...
 * Don't put the name of your driver there, it's useless. */

/* Basic operations */
#define SIOCSIWNWID	0x8B02		/* set network id (pre-802.11) */
#define SIOCGIWNWID	0x8B03		/* get network id (the cell) */
#define SIOCSIWFREQ	0x8B04		/* set channel/frequency (Hz) */
#define SIOCGIWFREQ	0x8B05		/* get channel/frequency (Hz) */
#define SIOCSIWMODE	0x8B06		/* set operation mode */
#define SIOCGIWMODE	0x8B07		/* get operation mode */
#define SIOCSIWSENS	0x8B08		/* set sensitivity (dBm) */
#define SIOCGIWSENS	0x8B09		/* get sensitivity (dBm) */

/* Informative stuff */
#define SIOCSIWRANGE	0x8B0A		/* Unused */
#define SIOCGIWRANGE	0x8B0B		/* Get range of parameters */
#define SIOCSIWPRIV	0x8B0C		/* Unused */
#define SIOCGIWPRIV	0x8B0D		/* get private ioctl interface info */
#define SIOCSIWSTATS	0x8B0E		/* Unused */
#define SIOCGIWSTATS	0x8B0F		/* Get /proc/net/wireless stats */
/* SIOCGIWSTATS is strictly used between user space and the kernel, and
 * is never passed to the driver (i.e. the driver will never see it). */

/* Spy support (statistics per MAC address - used for Mobile IP support) */
#define SIOCSIWSPY	0x8B10		/* set spy addresses */
#define SIOCGIWSPY	0x8B11		/* get spy info (quality of link) */
#define SIOCSIWTHRSPY	0x8B12		/* set spy threshold (spy event) */
#define SIOCGIWTHRSPY	0x8B13		/* get spy threshold */

/* Access Point manipulation */
#define SIOCSIWAP	0x8B14		/* set access point MAC addresses */
#define SIOCGIWAP	0x8B15		/* get access point MAC addresses */
#define SIOCGIWAPLIST	0x8B17		/* Deprecated in favor of scanning */
#define SIOCSIWSCAN	0x8B18		/* trigger scanning (list cells) */
#define SIOCGIWSCAN	0x8B19		/* get scanning results */

/* 802.11 specific support */
#define SIOCSIWESSID	0x8B1A		/* set ESSID (network name) */
#define SIOCGIWESSID	0x8B1B		/* get ESSID */
#define SIOCSIWNICKN	0x8B1C		/* set node name/nickname */
#define SIOCGIWNICKN	0x8B1D		/* get node name/nickname */
/* As the ESSID and NICKN are strings up to 32 bytes long, it doesn't fit
 * within the 'iwreq' structure, so we need to use the 'data' member to
 * point to a string in user space, like it is done for RANGE... */

/* Other parameters useful in 802.11 and some other devices */
#define SIOCSIWRATE	0x8B20		/* set default bit rate (bps) */
#define SIOCGIWRATE	0x8B21		/* get default bit rate (bps) */
#define SIOCSIWRTS	0x8B22		/* set RTS/CTS threshold (bytes) */
#define SIOCGIWRTS	0x8B23		/* get RTS/CTS threshold (bytes) */
#define SIOCSIWFRAG	0x8B24		/* set fragmentation thr (bytes) */
#define SIOCGIWFRAG	0x8B25		/* get fragmentation thr (bytes) */
#define SIOCSIWTXPOW	0x8B26		/* set transmit power (dBm) */
#define SIOCGIWTXPOW	0x8B27		/* get transmit power (dBm) */
#define SIOCSIWRETRY	0x8B28		/* set retry limits and lifetime */
#define SIOCGIWRETRY	0x8B29		/* get retry limits and lifetime */

/* Encoding stuff (scrambling, hardware security, WEP...) */
#define SIOCSIWENCODE	0x8B2A		/* set encoding token & mode */
#define SIOCGIWENCODE	0x8B2B		/* get encoding token & mode */
/* Power saving stuff (power management, unicast and multicast) */
#define SIOCSIWPOWER	0x8B2C		/* set Power Management settings */
#define SIOCGIWPOWER	0x8B2D		/* get Power Management settings */

/* WPA : Generic IEEE 802.11 informatiom element (e.g., for WPA/RSN/WMM).
 * This ioctl uses struct iw_point and data buffer that includes IE id and len
 * fields. More than one IE may be included in the request. Setting the generic
 * IE to empty buffer (len=0) removes the generic IE from the driver. Drivers
 * are allowed to generate their own WPA/RSN IEs, but in these cases, drivers
 * are required to report the used IE as a wireless event, e.g., when
 * associating with an AP. */
#define SIOCSIWGENIE	0x8B30		/* set generic IE */
#define SIOCGIWGENIE	0x8B31		/* get generic IE */

/* WPA : IEEE 802.11 MLME requests */
#define SIOCSIWMLME	0x8B16		/* request MLME operation; uses
					 * struct iw_mlme */
/* WPA : Authentication mode parameters */
#define SIOCSIWAUTH	0x8B32		/* set authentication mode params */
#define SIOCGIWAUTH	0x8B33		/* get authentication mode params */

/* WPA : Extended version of encoding configuration */
#define SIOCSIWENCODEEXT 0x8B34		/* set encoding token & mode */
#define SIOCGIWENCODEEXT 0x8B35		/* get encoding token & mode */

/* WPA2 : PMKSA cache management */
#define SIOCSIWPMKSA	0x8B36		/* PMKSA cache operation */

#define IWEVTXDROP	0x8C00		/* Packet dropped to excessive retry */
#define IWEVQUAL	0x8C01		/* Quality part of statistics (scan) */
#define IWEVCUSTOM	0x8C02		/* Driver specific ascii string */
#define IWEVREGISTERED	0x8C03		/* Discovered a new node (AP mode) */
#define IWEVEXPIRED	0x8C04		/* Expired a node (AP mode) */
#define IWEVGENIE	0x8C05		/* Generic IE (WPA, RSN, WMM, ..)*/

/* These 32 ioctl are wireless device private, for 16 commands.
 * Each driver is free to use them for whatever purpose it chooses,
 * however the driver *must* export the description of those ioctls
 * with SIOCGIWPRIV and *must* use arguments as defined below.
 * If you don't follow those rules, DaveM is going to hate you (reason :
 * it make mixed 32/64bit operation impossible).
 */
#define SIOCIWFIRSTPRIV	0x8BE0
#define SIOCIWLASTPRIV	0x8BFF
/* Channel flags. */
#define	IEEE80211_CHAN_TURBO	0x0010	/* Turbo channel */
#define	IEEE80211_CHAN_CCK	0x0020	/* CCK channel */
#define	IEEE80211_CHAN_OFDM	0x0040	/* OFDM channel */
#define	IEEE80211_CHAN_2GHZ	0x0080	/* 2 GHz spectrum channel. */
#define	IEEE80211_CHAN_5GHZ	0x0100	/* 5 GHz spectrum channel */
#define	IEEE80211_CHAN_PASSIVE	0x0200	/* Only passive scan allowed */
#define	IEEE80211_CHAN_DYN	0x0400	/* Dynamic CCK-OFDM channel */
#define	IEEE80211_CHAN_GFSK	0x0800	/* GFSK channel (FHSS PHY) */

/* For IEEE80211_RADIOTAP_FLAGS */
#define	IEEE80211_RADIOTAP_F_CFP	0x01	/* sent/received
						 * during CFP
						 */
#define	IEEE80211_RADIOTAP_F_SHORTPRE	0x02	/* sent/received
						 * with short
						 * preamble
						 */
#define	IEEE80211_RADIOTAP_F_WEP	0x04	/* sent/received
						 * with WEP encryption
						 */
#define	IEEE80211_RADIOTAP_F_FRAG	0x08	/* sent/received
						 * with fragmentation
						 */
#define	IEEE80211_RADIOTAP_F_FCS	0x10	/* frame includes FCS */
#define	IEEE80211_RADIOTAP_F_DATAPAD	0x20	/* frame has padding between
						 * 802.11 header and payload
						 * (to 32-bit boundary)
						 */
#define IEEE80211_RADIOTAP_F_BADFCS	0x40	/* bad FCS */

/* For IEEE80211_RADIOTAP_RX_FLAGS */
#define IEEE80211_RADIOTAP_F_RX_BADPLCP	0x0002	/* frame has bad PLCP */

/* For IEEE80211_RADIOTAP_TX_FLAGS */
#define IEEE80211_RADIOTAP_F_TX_FAIL	0x0001	/* failed due to excessive
						 * retries */
#define IEEE80211_RADIOTAP_F_TX_CTS	0x0002	/* used cts 'protection' */
#define IEEE80211_RADIOTAP_F_TX_RTS	0x0004	/* used rts/cts handshake */


/* For IEEE80211_RADIOTAP_MCS */
#define IEEE80211_RADIOTAP_MCS_HAVE_BW		0x01
#define IEEE80211_RADIOTAP_MCS_HAVE_MCS		0x02
#define IEEE80211_RADIOTAP_MCS_HAVE_GI		0x04
#define IEEE80211_RADIOTAP_MCS_HAVE_FMT		0x08
#define IEEE80211_RADIOTAP_MCS_HAVE_FEC		0x10

#define IEEE80211_RADIOTAP_MCS_BW_MASK		0x03
#define		IEEE80211_RADIOTAP_MCS_BW_20	0
#define		IEEE80211_RADIOTAP_MCS_BW_40	1
#define		IEEE80211_RADIOTAP_MCS_BW_20L	2
#define		IEEE80211_RADIOTAP_MCS_BW_20U	3
#define IEEE80211_RADIOTAP_MCS_SGI		0x04
#define IEEE80211_RADIOTAP_MCS_FMT_GF		0x08
#define IEEE80211_RADIOTAP_MCS_FEC_LDPC		0x10


/* Ugly macro to convert literal channel numbers into their mhz equivalents
 * There are certianly some conditions that will break this (like feeding it '30')
 * but they shouldn't arise since nothing talks on channel 30. */
#define ieee80211chan2mhz(x) \
	(((x) <= 14) ? \
	(((x) == 14) ? 2484 : ((x) * 5) + 2407) : \
	((x) + 1000) * 5)


typedef u16  __sum16;
typedef u32  __wsum;

#define CHECKSUM_NONE 0
#define CHECKSUM_UNNECESSARY 1
#define CHECKSUM_COMPLETE 2
#define CHECKSUM_PARTIAL 3


#define smp_rmb()

#define atomic_t atomic

#define I802_DEBUG_INC(x)

#define secpath_get(x) (x)

#define BUG() p_dbg("bug\n")

struct sk_buff *skb_copy(const struct sk_buff *skb, gfp_t gfp_mask);
int skb_pad(struct sk_buff *skb, int pad);
unsigned char *skb_put(struct sk_buff *skb, unsigned int len);


//#define dev_kfree_skb(a)	consume_skb(a)

#define wiphy_debug(x,...) p_dbg(__VA_ARGS__)

#define lockdep_assert_held(x)

#define SKB_LINEAR_ASSERT(x)

//#define spin_lock_bh(x)
//#define spin_unlock_bh(x)

#define might_sleep()
#define smp_mb()
#define barrier()

#define WARN_ON_ONCE(x) 
#define rcu_assign_pointer(x,y) 
#define ASSERT_RTNL()

#define rtnl_dereference(x) (x)

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define ror16(x, y)	(((x) >> (y)) | ((x) << (16 - (y))))

//#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

#define jiffies_to_msecs(x)	((x)*10)

struct list_head {
	struct list_head *next, *prev;
};

struct list {
	struct list *next, *prev;
};

#define HZ 100

//#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#define container_of(ptr, type, member) (type *)( (char *)ptr - offsetof(type,member) )
/*

#define container_of(ptr, type, member) ({                      \
	const ( ((type *)0)->member ) *__mptr = (ptr);    \
	(type *)( (char *)__mptr - offsetof(type,member) );})
*/
#define list_entry(ptr, type, member) \
	container_of(ptr, type, member)

#define list_for_each_entry(pos, head, member)				\
	for (pos = list_entry((head)->next, (*pos), member);	\
	     &pos->member != (head); 	\
	     pos = list_entry(pos->member.next, (*pos), member))


struct netdev_hw_addr_list {
	struct list_head	list;
	int			count;
};

struct timespec {
	long		tv_sec;			/* seconds */
	long		tv_nsec;		/* nanoseconds */
};

struct ewma {
	unsigned long internal;
	unsigned long factor;
	unsigned long weight;
};

//#define MAX_SKB_FRAGS  4

//typedef struct skb_frag_struct skb_frag_t;

struct ts_config
{
//	struct ts_ops		*ops;
	int 			flags;

	/**
	 * get_next_block - fetch next block of data
	 * @consumed: number of bytes consumed by the caller
	 * @dst: destination buffer
	 * @conf: search configuration
	 * @state: search state
	 *
	 * Called repeatedly until 0 is returned. Must assign the
	 * head of the next block of data to &*dst and return the length
	 * of the block or 0 if at the end. consumed == 0 indicates
	 * a new search. May store/read persistent values in state->cb.
	 */
	unsigned int		(*get_next_block)(unsigned int consumed,
						  const u8 **dst,
						  struct ts_config *conf,
						  void *state);

	/**
	 * finish - finalize/clean a series of get_next_block() calls
	 * @conf: search configuration
	 * @state: search state
	 *
	 * Called after the last use of get_next_block(), may be used
	 * to cleanup any leftovers.
	 */
	void			(*finish)(struct ts_config *conf,
					  void *state);
};

#define PACKET_HOST		0		/* To us		*/
#define PACKET_BROADCAST	1		/* To all		*/
#define PACKET_MULTICAST	2		/* To group		*/
#define PACKET_OTHERHOST	3		/* To someone else 	*/
#define PACKET_OUTGOING		4		/* Outgoing of any type */
/* These ones are invisible by user level */
#define PACKET_LOOPBACK		5		/* MC/BRD frame looped back */
#define PACKET_FASTROUTE	6		/* Fastrouted frame	*/


void do_posix_clock_monotonic_gettime(struct timespec *t);
unsigned long ewma_read(const struct ewma *avg);
//unsigned int cfg80211_classify8021d(struct sk_buff *skb);
void * ERR_PTR(long error);
long PTR_ERR(const void *ptr);
long IS_ERR(const void *ptr);
long IS_ERR_OR_NULL(const void *ptr);
void *wdev_priv(struct wireless_dev *wdev);

//int skb_tailroom(const struct sk_buff *skb);
//unsigned int skb_headroom(const struct sk_buff *skb);

__be16 eth_type_trans(struct sk_buff *skb, struct net_device *dev);
char *strsep(char **s, const char *ct);
unsigned int usb_sndctrlpipe(VOID *dev, ULONG address);
unsigned int usb_rcvctrlpipe(VOID *dev, ULONG address);

UINT16 RtmpOsGetUnaligned(UINT16 *pWord);

void RtmpOsPutUnaligned(UINT16 val, UINT16 *pWord);
void RtmpOsPutUnaligned32(UINT32 val, UINT32 *pWord);
UINT32 RtmpOsGetUnaligned32(UINT32 *pWord);

int ieee80211_channel_to_frequency(int chan, enum ieee80211_band band);

#endif

