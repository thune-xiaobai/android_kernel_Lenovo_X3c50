#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/types.h>
#include <linux/jhash.h>
#include <linux/jiffies.h>
#include <linux/ctype.h>
#include <net/ip.h>

#include "bwlist_public.h"
#include "bwlist_comm.h"
#include "hashtb_comm.h"


// from RFC
#define MAX_DOMAIN_LEN 67
#define DNS_TYPE_A      1
#define DNS_TYPE_CNAME  5



/* 暂未支持DNS代理 */

nf_hookfn dns_pre_v4;
nf_hookfn dns_pre_v6;

static struct nf_hook_ops dns_hook_ops[] = {
    {
        .hook = dns_pre_v4,
        .owner = THIS_MODULE,
        .pf = NFPROTO_IPV4,
        .hooknum = NF_INET_PRE_ROUTING,
        .priority = NF_IP_PRI_LAST,
    },
    {
        .hook = dns_pre_v6,
        .owner = THIS_MODULE,
        .pf = NFPROTO_IPV6,
        .hooknum = NF_INET_PRE_ROUTING,
        .priority = NF_IP_PRI_LAST,
    },
};

struct dns_hdr {
    u16                 tid;
    u16                 flags;
    u16                 questions;
    u16                 answer_rrs;
    u16                 auth_rrs;
    u16                 addi_rrs;
};

struct dns_req_hdr {
    u16                 type;
    u16                 class;
};

#pragma pack(1)
struct rr_data {
    struct dns_req_hdr  t_c;
    uint32_t            ttl;
    uint16_t            len;
    char                data[0];
} __attribute__ ((aligned(1)));
#pragma pack()

int bwlist_add_dnsip(const unsigned char *domain, uint32_t h_ip, unsigned long expire_jiff);

/* 支持压缩和多次嵌套 */
int parse_domain(char domain[MAX_DOMAIN_LEN + 1], int offset, int need_domain,
                 const unsigned char *data, int datalen,
                 const unsigned char *dns_head, const int dns_len)
{
    int idx;
    int len;
    const unsigned char *ptr;
    int walk_forward = 0;

    if(offset > MAX_DOMAIN_LEN) {
        bwlist_debug("bad offset: %d\n", offset);
        return -1;
    }

    ptr = data;
    idx = offset;
    while(1) {
        len = *ptr++;
        if(len == 0) {
            if(idx > 1 && domain[idx - 1] == '.') {
                domain[idx - 1] = '\0';
            } else {
                domain[idx] = '\0';
            }
            break;
        }

        if((len & 0xc0) == 0xc0) {
            len = ((len & 0x3f) << 8) | *ptr++;
            if(len > dns_len) {
                bwlist_debug("len: %d, [0x%x, 0x%x], ptr: 0x%p\n", len, *(ptr - 2), *(ptr - 1), ptr);
                goto err;
            }

            if(need_domain) {
                len = parse_domain(domain, idx, need_domain,
                                   dns_head + len, dns_len - len,
                                   dns_head, dns_len);
                if(len < 0) {
                    bwlist_debug("parse compressive domain failed\n");
                    goto err;
                }
            }
            break;
        } else {
            if(len > datalen) {
                bwlist_debug("bad len: %d, datalen: %d\n", len, datalen);
                goto err;
            }

            if(idx + len + 1 <= MAX_DOMAIN_LEN) {
                if(need_domain) {
                    memcpy(domain + idx, ptr, len);
                    idx += len;

                    *(domain + idx) = '.';
                    idx++;
                } else {
                    idx += len + 1;
                }
            }

            ptr += len;
            datalen -= len;

            walk_forward += len;
        }
    }

    return ptr - data;

err:
    domain[0] = '\0';
    return -1;
}

int dns_parse_question(char domain[MAX_DOMAIN_LEN + 1], unsigned char *data, int datalen,
                       const unsigned char *dns_data, const int dns_len)
{
    struct dns_req_hdr *reqh;
    unsigned char *ptr;
    int len;

    ptr = data;
    len = parse_domain(domain, 0, 1, ptr, datalen, dns_data, dns_len);
    if(len < 0) {
        bwlist_debug("parse domain failed\n");
        return -1;
    }

    ptr += len;
    datalen -= len;

    if(datalen < sizeof(struct dns_req_hdr)) {
        bwlist_debug("bad datalen: %d\n", datalen);
        return -1;
    }

    reqh = (struct dns_req_hdr *)ptr;
    if(reqh->type != __constant_htons(DNS_TYPE_A)) {
        bwlist_debug("not DNS_TYPE_A, type: %d\n", ntohs(reqh->type));
        return -1;
    }

    return (unsigned char *)reqh + sizeof(*reqh) - data;
}



int dns_parse_answer(const unsigned char *question,
                     unsigned char *data, int datalen,
                     const unsigned char *dns_data, const int dns_len)
{
    struct rr_data *rr;
    int ret;
    int a_cnt;
    char domain[MAX_DOMAIN_LEN + 1];
    uint32_t h_max_ttl = 0, h_rr_ttl;

    bwlist_debug("dns_parse_answer in, datalen: %d, dns_len: %d\n", datalen, dns_len);
    
    a_cnt = 0;
    while(datalen > 0) {
        ret = parse_domain(domain, 0, 1, data, datalen, dns_data, dns_len);
        if(ret < 0) {
            bwlist_debug("parse domain failed\n");
            goto err;
        }

        data += ret;
        datalen -= ret;

        if(datalen < sizeof(struct rr_data)) {
            bwlist_debug("bad len: %d\n", datalen);
            goto err;
        }
        rr = (struct rr_data *)data;
        if(datalen < sizeof(struct rr_data) + ntohs(rr->len)) {
            bwlist_debug("bad len: %d, rr->len: %d\n", datalen, ntohs(rr->len));
            goto err;
        }

        if(rr->t_c.type == __constant_htons(DNS_TYPE_CNAME)) {
            h_rr_ttl = ntohl(rr->ttl);
            if(h_max_ttl < h_rr_ttl) {
                h_max_ttl = h_rr_ttl;
            }
            bwlist_debug("CNAME, cdomain: %s, ttl: %u, max_ttl: %u\n", domain, rr->ttl, h_max_ttl);
        } else if(rr->t_c.type == __constant_htons(DNS_TYPE_A)) {
            uint32_t nip = *(uint32_t *)(data + sizeof(struct rr_data));
            unsigned long jiff;

            h_rr_ttl = ntohl(rr->ttl);
            jiff = (h_max_ttl >= h_rr_ttl ? h_max_ttl : h_rr_ttl) * HZ + jiffies;

            bwlist_add_dnsip(question, ntohl(nip), jiff);
            bwlist_debug("A: %u.%u.%u.%u, domain: %s, ttl: %u, max_ttl: %u\n",
                         NIPQUAD(nip), question, h_rr_ttl, h_max_ttl);
        } else {
            // do something?
            bwlist_debug("other type: %d\n", ntohs(rr->t_c.type));
        }

        data += sizeof(struct rr_data) + ntohs(rr->len);
        datalen -= sizeof(struct rr_data) + ntohs(rr->len);
    }

    return 0;

err:
    return -1;
}

void dns_do_parse(unsigned char *data, int datalen)
{
    struct dns_hdr *dhdr;
    char domain[MAX_DOMAIN_LEN + 1];
    int ret, offset;
    int i;

    dhdr = (struct dns_hdr *)data;
    if(!(ntohs(dhdr->flags) & 0x8000)) {
        bwlist_debug("not response?\n");
        return ;    // not response
    }
    if((ntohs(dhdr->flags) & 0x000f) != 0) {
        bwlist_debug("error ?\n");
        return ;    // error
    }

    offset = sizeof(struct dns_hdr);
    ret = dns_parse_question(domain, data + offset, datalen - offset, data, datalen);
    if(ret < 0) {
        return ;
    }

    for(i = 0; i < strlen(domain); i++) {
        domain[i] = tolower(domain[i]);
    }

    offset += ret;
    if(dns_parse_answer(domain, data + offset, datalen - offset, data, datalen) < 0) {
        return ;
    }
}

void dns_pre_handle(struct sk_buff *skb)
{
    struct iphdr *iph;
    struct udphdr *udph;
    unsigned char *dns_data;
    int datalen;

    iph = (struct iphdr *)skb_network_header(skb);

    if(iph->protocol == IPPROTO_UDP) {
        udph = (struct udphdr *)((unsigned char*)iph + (iph->ihl << 2));
#define DNS_PORT (53)
        if(udph->source == __constant_htons(DNS_PORT) || udph->dest == __constant_htons(DNS_PORT)) {
            dns_data = (unsigned char *)udph + sizeof(*udph);
            datalen = ntohs(iph->tot_len) - (iph->ihl << 2) - sizeof(*udph);

            //hex_printout("dns", dns_data, datalen);

            dns_do_parse(dns_data, datalen);
        }
    }
}

unsigned int dns_pre_v4(
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
    unsigned int hooknum,
#else
    const struct nf_hook_ops *ops,
#endif
    struct sk_buff *skb,
    const struct net_device *in,
    const struct net_device *out,
    int (*okfn)(struct sk_buff *))
{
    if(is_bwlist_on()) {
        dns_pre_handle(skb);
    }
    return NF_ACCEPT;
}

unsigned int dns_pre_v6(
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
    unsigned int hooknum,
#else
    const struct nf_hook_ops *ops,
#endif
    struct sk_buff *skb,
    const struct net_device *in,
    const struct net_device *out,
    int (*okfn)(struct sk_buff *))
{
    return NF_ACCEPT;
}


typedef struct bwlist_dnsIP {
    uint32_t                        h_ip;
    uint32_t                        grp_magic;
    unsigned long                   expire_time; // jiffies
    bwlist_group_t                  *grp;
} bwlist_dnsIP_t;

#define DNSIP_NODE_CHECKTIME        (HZ)
#define DNSIP_EXPIRE_TIME_RESERVED  (300*HZ)

static hashtb_t *g_dnsIP_hashtb;
static unsigned long g_dnsIP_expire_time = DNSIP_EXPIRE_TIME_RESERVED;

void bwlist_dnsIP_set_expire_time(unsigned long seconds)
{
    g_dnsIP_expire_time = seconds * HZ;
}

unsigned long bwlist_dnsIP_get_expire_time(void)
{
    return g_dnsIP_expire_time / HZ;
}

static void bwlist_dnsIP_node_dump(void *node_data)
{
    bwlist_dnsIP_t *node = (bwlist_dnsIP_t*)node_data;

    bwlist_debug("dnsIP: %u.%u.%u.%u, expire_time: %lu, redirect_url: %s\n",
                 HIPQUAD(node->h_ip), node->expire_time,
                 node->grp ? node->grp->redirect_url : "group is null");
}

static void bwlist_dnsIP_node_cleanup(void *node_data)
{
    bwlist_debug("free one dnsIP node\n");
}

static int bwlist_dnsIP_node_key_match(void *node_data1, void *node_data2)
{
    bwlist_dnsIP_t *node1, *node2;

    node1 = (bwlist_dnsIP_t *)node_data1;
    node2 = (bwlist_dnsIP_t *)node_data2;

    return node1->h_ip == node2->h_ip;
}

static int bwlist_dnsIP_node_value_match(void *node_data1, void *node_data2)
{
    bwlist_dnsIP_t *node1, *node2;

    node1 = (bwlist_dnsIP_t *)node_data1;
    node2 = (bwlist_dnsIP_t *)node_data2;
    return node1->grp == node2->grp && node1->expire_time == node2->expire_time;
}

static uint32_t bwlist_dnsIP_node_hash_key(void *node_data, uint32_t size)
{
    bwlist_dnsIP_t *node = (bwlist_dnsIP_t*)node_data;
    return jhash_1word(node->h_ip, 0);
}

static inline int is_expired(unsigned long jiff, bwlist_dnsIP_t *node)
{
    if(time_after(jiff, node->expire_time + g_dnsIP_expire_time)) {
        return 1;
    }
    return 0;
}

static int bwlist_dnsIP_node_timeout_check(void *node_data)
{
    bwlist_dnsIP_t *node = (bwlist_dnsIP_t *)node_data;

    return is_expired(jiffies, node);
}


int bwlist_add_dnsip(const unsigned char *domain, uint32_t h_ip, unsigned long expire_jiff)
{
    bwlist_group_t *grp;
    bwlist_dnsIP_t *node;
    bwlist_dnsIP_t input = {
        .h_ip = h_ip,
    };
    int ret;

    uint32_t new_grp_magic = bwlist_grp_magic();

    rcu_read_lock();
    node = ht_node_find(g_dnsIP_hashtb, &input, sizeof(input));
    if(node) {
        if(new_grp_magic == node->grp_magic) {
            if(!is_expired(expire_jiff, node)) {
                rcu_read_unlock();
                return 0;
            }
            grp = node->grp;
        } else {
            grp = bwlist_find_grp_with_key(domain);
        }
    } else {
        grp = bwlist_find_grp_with_key(domain);
    }

    node = ht_data_create(g_dnsIP_hashtb);
    if(!node) {
        rcu_read_unlock();
        return -1;
    }
    node->grp = grp; // NULL is allowed.
    node->grp_magic = new_grp_magic;
    node->expire_time = expire_jiff;
    node->h_ip = h_ip;

    ret = ht_node_update(g_dnsIP_hashtb, node, sizeof(*node));
    rcu_read_unlock();

    if(ret < 0) {
        bwlist_debug("ht_node_update error, errno is %d\n", ret);

        ht_data_destroy(g_dnsIP_hashtb, node, sizeof(*node));
        return -1;
    }
    bwlist_debug("add ok, %u.%u.%u.%u, expired: %lu, magic: %u\n", HIPQUAD(h_ip), expire_jiff, new_grp_magic);
    return 0;
}

bwlist_group_t *__bwlist_find_grp_with_dnsIP(uint32_t h_dst)
{
    bwlist_dnsIP_t *node;
    bwlist_dnsIP_t input = {
        .h_ip = h_dst,
    };
    uint32_t new_grp_magic = bwlist_grp_magic();

    node = ht_node_find(g_dnsIP_hashtb, &input, sizeof(input));
    if(!node) {
        bwlist_debug("find dns ip: %u.%u.%u.%u failed, not found\n", HIPQUAD(h_dst));
        return NULL;
    }

    if(node->grp_magic != new_grp_magic) {
        bwlist_debug("find dns ip: %u.%u.%u.%u failed, grp_magic invalid\n", HIPQUAD(h_dst));
        bwlist_debug("old_grp_magic: %u, new_grp_magic: %u\n", node->grp_magic, new_grp_magic);
        return NULL;
    }

    if(is_expired(jiffies, node)) {
        bwlist_debug("find dns ip: %u.%u.%u.%u failed, expired\n", HIPQUAD(h_dst));
        return NULL;
    }

    bwlist_debug("find dns ip: %u.%u.%u.%u, matched grp: %p\n", HIPQUAD(h_dst), node->grp);
    return node->grp;
}

void bwlist_dnsIP_dump(void)
{
    int cnt = ht_dump_all(g_dnsIP_hashtb);
    bwlist_debug("dump dnsIP node cnt: %d\n", cnt);
}

void bwlist_dnsIP_invalid_all(void)
{
    int cnt = ht_cleanup_nodes(g_dnsIP_hashtb);
    bwlist_debug("invalid dnsIP node cnt: %d\n", cnt);
}

int bwlist_dnsIP_node_cnt(void)
{
    return ht_node_cnt(g_dnsIP_hashtb);
}

int bwlist_dnsIP_init(int max_dnsIP_cnt)
{
    hashtb_t *me;

    me = ht_create("dns_cache", max_dnsIP_cnt, sizeof(bwlist_dnsIP_t),
                   DNSIP_NODE_CHECKTIME, max_dnsIP_cnt / 60,
                   bwlist_dnsIP_node_key_match, bwlist_dnsIP_node_value_match,
                   bwlist_dnsIP_node_hash_key, bwlist_dnsIP_node_timeout_check,
                   bwlist_dnsIP_node_cleanup, bwlist_dnsIP_node_dump);
    if(!me) {
        bwlist_debug("dns ht create failed\n");
        return -1;
    }
    g_dnsIP_hashtb = me;


    if(nf_register_hooks(dns_hook_ops, ARRAY_SIZE(dns_hook_ops)) < 0) {
        bwlist_debug("hook register failed\n");
        ht_destroy(g_dnsIP_hashtb);
        return -1;
    }
    return 0;
}

void bwlist_dnsIP_fini(void)
{
    nf_unregister_hooks(dns_hook_ops, ARRAY_SIZE(dns_hook_ops));

    ht_destroy(g_dnsIP_hashtb);

    bwlist_debug("bwlist_dns_fini over...\n");
}

