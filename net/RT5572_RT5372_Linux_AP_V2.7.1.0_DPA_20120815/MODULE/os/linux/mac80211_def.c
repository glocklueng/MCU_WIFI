#define DEBUG
#include "rt_config.h"
#include "mac80211_def.h"
#include "string.h"

#define MAC80211_PAD_GET(__pAd, __pWiphy)							\
{																\
__pAd = (ULONG *)(wiphy_priv(__pWiphy));					\
if (__pAd == NULL)											\
{															\
DBGPRINT(RT_DEBUG_ERROR,								\
("80211> %s but pAd = NULL!", __FUNCTION__));	\
return -EINVAL;											\
}															\
}


int copy_from_user(void *dst, void *src, int size)
{
	memcpy(dst, src, size);
	return 0;
}

#if 0
unsigned char *skb_transport_header(const struct sk_buff *skb)
{
	return skb->transport_header;
} 

void skb_reset_transport_header(struct sk_buff *skb)
{
	skb->transport_header = skb->data;
} 

void skb_set_transport_header(struct sk_buff *skb, const int offset)
{
	skb->transport_header = skb->data + offset;
} 

unsigned char *skb_network_header(const struct sk_buff *skb)
{
	return skb->network_header;
} 

void skb_reset_network_header(struct sk_buff *skb)
{
	skb->network_header = skb->data;
} 

void skb_set_network_header(struct sk_buff *skb, const int offset)
{
	skb->network_header = skb->data + offset;
} 

unsigned char *skb_mac_header(const struct sk_buff *skb)
{
	return skb->mac_header;
} 

int skb_mac_header_was_set(const struct sk_buff *skb)
{
	return skb->mac_header != NULL;
} 

void skb_reset_mac_header(struct sk_buff *skb)
{
	skb->mac_header = skb->data;
} 

void skb_set_mac_header(struct sk_buff *skb, const int offset)
{
	skb->mac_header = skb->data + offset;
} 

void skb_copy_from_linear_data(const struct sk_buff *skb, void *to, const
	unsigned int len)
{
	memcpy(to, skb->data, len);
} 

void skb_copy_from_linear_data_offset(const struct sk_buff *skb, const int
	offset, void *to, const unsigned int len)
{
	memcpy(to, skb->data + offset, len);
} 

void skb_copy_to_linear_data(struct sk_buff *skb, const void *from, const
	unsigned int len)
{
	memcpy(skb->data, from, len);
} 

void skb_copy_to_linear_data_offset(struct sk_buff *skb, const int offset,
	const void *from, const unsigned int len)
{
	memcpy(skb->data + offset, from, len);
} 

unsigned char *skb_tail_pointer(const struct sk_buff *skb)
{
	return skb->tail;
} 

void skb_reset_tail_pointer(struct sk_buff *skb)
{
	skb->tail = skb->data;
} 

void skb_set_tail_pointer(struct sk_buff *skb, const int offset)
{
	skb->tail = skb->data + offset;
} 


unsigned int cfg80211_classify8021d(struct sk_buff *skb)
{
	unsigned int dscp;

	/* skb->priority values from 256->263 are magic values to
	 * directly indicate a specific 802.1d priority.  This is used
	 * to allow 802.1d priority to be passed directly in from VLAN
	 * tags, etc.
	 */
	if (skb->priority >= 256 && skb->priority <= 263)
		return skb->priority - 256;

	switch (skb->protocol)
	{
		case htons(ETH_P_IP): dscp = ip_hdr(skb)->tos &0xfc;
		break;
		default:
			return 0;
	}

	return dscp >> 5;

}

#endif 

#define MAX_ERRNO	4095

#define IS_ERR_VALUE(x) unlikely((x) >= (unsigned long)-MAX_ERRNO)


char *strsep(char **s, const char *ct)
{
	char *sbegin =  *s;
	char *end;

	if (sbegin == NULL)
		return NULL;

	end = strpbrk(sbegin, ct);
	if (end)
		*end++ = '\0';
	*s = end;
	return sbegin;
}

UINT16 RtmpOsGetUnaligned(IN UINT16 *pWord)
{
	UINT16 val;
	memcpy(&val, pWord, 2);
	return val;
}


void RtmpOsPutUnaligned(UINT16 val, UINT16 *pWord)
{
	memcpy(pWord, &val, 2);
}


UINT32 RtmpOsGetUnaligned32(IN UINT32 *pWord)
{
	UINT32 val;
	memcpy(&val, pWord, 4);
	return val;
}

void RtmpOsPutUnaligned32(UINT32 val, UINT32 *pWord)
{
	memcpy(pWord, &val, 4);
}


ULONG RtmpOsGetUnalignedlong(IN ULONG *pWord)
{
	UINT32 val;
	memcpy(&val, pWord, 4);
	return val;
}

void RtmpOsPutUnalignedlong(UINT32 val, UINT32 *pWord)
{
	memcpy(pWord, &val, 4);
}


unsigned int usb_sndctrlpipe(VOID *dev, ULONG address)
{
	return 0;
}

unsigned int usb_rcvctrlpipe(VOID *dev, ULONG address)
{
	return 0;
}

void wsum_memmove(void *dst, void *src, int size)
{
	char *_dst = (char*)dst;
	char *_src = (char*)src;
	int i;
	if (!dst || !src)
		return ;

	for (i = 0; i < size; i++)
		_dst[i] = _src[i];
}


int ieee80211_channel_to_frequency(int chan, enum ieee80211_band band)
{
	/* see 802.11 17.3.8.3.2 and Annex J
	 * there are overlapping channel numbers in 5GHz and 2GHz bands */
	if (band == IEEE80211_BAND_5GHZ)
	{
		if (chan >= 182 && chan <= 196)
			return 4000+chan * 5;
		else
			return 5000+chan * 5;
	} 
	else
	{
		 /* IEEE80211_BAND_2GHZ */
		if (chan == 14)
			return 2484;
		else if (chan < 14)
			return 2407+chan * 5;
		else
			return 0;
		 /* not supported */
	}
}

void *ERR_PTR(long error)
{
	return (void*)error;
}

long PTR_ERR(const void *ptr)
{
	return (long)ptr;
}

long IS_ERR(const void *ptr)
{
	return IS_ERR_VALUE((unsigned long)ptr);
}

long IS_ERR_OR_NULL(const void *ptr)
{
	return !ptr || IS_ERR_VALUE((unsigned long)ptr);
}

void do_posix_clock_monotonic_gettime(struct timespec *t)
{
	t->tv_nsec = jiffies * 10 * 1000000;
	t->tv_sec = jiffies / 100;
} 

unsigned long ewma_read(const struct ewma *avg)
{
	return avg->internal >> avg->factor;
} 

void *wiphy_priv(struct wiphy *wiphy)
{
	return wiphy->priv;
} 

/*
void *wdev_priv(struct wireless_dev *wdev)
{
return wiphy_priv(wdev->wiphy);
}*/


void dev_kfree_skb_any(struct sk_buff *skb)
{
	dev_kfree_skb(skb);
} 

static unsigned compare_ether_addr(const u8 *addr1, const u8 *addr2)
{
	const u16 *a = (const u16*)addr1;
	const u16 *b = (const u16*)addr2;

	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2])) != 0;
}

static struct ethhdr *eth_hdr(const struct sk_buff *skb)
{
	return (struct ethhdr*)skb_mac_header(skb);
} 

static unsigned compare_ether_addr_64bits(const u8 addr1[6+2], const u8 addr2[6
	+2])
{
	return compare_ether_addr(addr1, addr2);

}

static bool netdev_uses_dsa_tags(struct net_device *dev)
{
	return 0;
} 


__be16 eth_type_trans(struct sk_buff *skb, struct net_device *dev)
{
	struct ethhdr *eth;

	skb->dev = dev;
	skb_reset_mac_header(skb);
	skb_pull_inline(skb, ETH_HLEN);
	eth = eth_hdr(skb);

	if (unlikely(is_multicast_ether_addr(eth->h_dest)))
	{
		if (!compare_ether_addr_64bits(eth->h_dest, dev->broadcast))
			skb->pkt_type = PACKET_BROADCAST;
		else
			skb->pkt_type = PACKET_MULTICAST;
	} 

	/*
	 *      This ALLMULTI check should be redundant by 1.4
	 *      so don't forget to remove it.
	 *
	 *      Seems, you forgot to remove it. All silly devices
	 *      seems to set IFF_PROMISC.
	 */

	else if (1 /*dev->flags&IFF_PROMISC */)
	{
		if (unlikely(compare_ether_addr_64bits(eth->h_dest, dev->dev_addr)))
			skb->pkt_type = PACKET_OTHERHOST;
	}

	/*
	 * Some variants of DSA tagging don't have an ethertype field
	 * at all, so we check here whether one of those tagging
	 * variants has been configured on the receiving interface,
	 * and if so, set skb->protocol without looking at the packet.
	 */
	//	if (netdev_uses_dsa_tags(dev))
	//		return htons(ETH_P_DSA);
	//	if (netdev_uses_trailer_tags(dev))
	//		return htons(ETH_P_TRAILER);

	if (ntohs(eth->h_proto) >= 1536)
		return eth->h_proto;

	/*
	 *      This is a magic hack to spot IPX packets. Older Novell breaks
	 *      the protocol design and runs IPX over 802.3 without an 802.2 LLC
	 *      layer. We look for FFFF which isn't a used 802.2 SSAP/DSAP. This
	 *      won't work for fault tolerant netware but does for the rest.
	 */
	if (skb->len >= 2 && *(unsigned short*)(skb->data) == 0xFFFF)
		return htons(ETH_P_802_3);

	/*
	 *      Real 802.2 LLC
	 */
	return htons(ETH_P_802_2);
}

extern struct netif *p_netif;
int netif_rx(struct sk_buff *skb)
{

	ethernetif_input(p_netif, skb_mac_header(skb), skb->len + sizeof(struct ethhdr));
	kfree_skb(skb);
	return  0;
} 

void skb_check(void *buff)
{
	//struct sk_buff *skb = (struct sk_buff *)buff;
	//mem_check(skb->head);
}

int netif_rx_ni(struct sk_buff *skb)
{
	int err =  - 1;
	p_dbg_enter;
	/*
	preempt_disable();
	err = netif_rx(skb);
	if (local_softirq_pending())
	do_softirq();
	preempt_enable();
	 */
	return err;
} 

VOID initList(IN PLIST_HEADER pList)
{
	pList->pHead = pList->pTail = NULL;
	pList->size = 0;
	return ;
}

VOID insertTailList(IN PLIST_HEADER pList, IN PLIST_ENTRY pEntry)
{
	uint32_t cpu_sr;
	ASSERT(pList);
    	cpu_sr = local_irq_save();
	pEntry->pNext = NULL;
	if (pList->pTail)
		pList->pTail->pNext = pEntry;
	else
		pList->pHead = pEntry;
	pList->pTail = pEntry;
	pList->size++;
	local_irq_restore(cpu_sr);
	return ;
}

PLIST_ENTRY removeHeadList(IN PLIST_HEADER pList)
{
	PLIST_ENTRY pNext;
	PLIST_ENTRY pEntry = 0;
	uint32_t cpu_sr;
	ASSERT(pList);
    	cpu_sr = local_irq_save();
	pEntry = pList->pHead;
	if (pList->pHead != NULL)
	{
		pNext = pList->pHead->pNext;
		pList->pHead = pNext;
		if (pNext == NULL)
			pList->pTail = NULL;
		pList->size--;
	}else{
		
		pList->size = 0;
	}
	local_irq_restore(cpu_sr);
	return pEntry;
}

int getListSize(IN PLIST_HEADER pList)
{
	return pList->size;
}

PLIST_ENTRY delEntryList(IN PLIST_HEADER pList, IN PLIST_ENTRY pEntry)
{
	PLIST_ENTRY pCurEntry;
	PLIST_ENTRY pPrvEntry;

	if (pList->pHead == NULL)
		return NULL;

	if (pEntry == pList->pHead)
	{
		pCurEntry = pList->pHead;
		pList->pHead = pCurEntry->pNext;

		if (pList->pHead == NULL)
			pList->pTail = NULL;

		pList->size--;
		return pCurEntry;
	}

	pPrvEntry = pList->pHead;
	pCurEntry = pPrvEntry->pNext;
	while (pCurEntry != NULL)
	{
		if (pEntry == pCurEntry)
		{
			pPrvEntry->pNext = pCurEntry->pNext;

			if (pEntry == pList->pTail)
				pList->pTail = pPrvEntry;

			pList->size--;
			break;
		}
		pPrvEntry = pCurEntry;
		pCurEntry = pPrvEntry->pNext;
	}

	return pCurEntry;
}

int ieee80211_frequency_to_channel(int freq)
{
	/* see 802.11 17.3.8.3.2 and Annex J */
	if (freq == 2484)
		return 14;
	else if (freq < 2484)
		return (freq - 2407) / 5;
	else if (freq >= 4910 && freq <= 4980)
		return (freq - 4000) / 5;
	else
		return (freq - 5000) / 5;
}

uint32_t  OS_SEM_LOCK_IRQ_FLAG = 0;
