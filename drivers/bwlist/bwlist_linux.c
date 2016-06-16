#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/arp.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/etherdevice.h>
#include <linux/ctype.h>

#include "bwlist_public.h"
#include "bwlist_ioctl.h"
#include "bwlist_sess.h"
#include "bwlist_comm.h"
#include "bwlist_dnscache.h"


#define HTTP_PREFEX         "http://"
#define WWW_PREFIX          "www."


//重定向相关
#define MAX_DATA_SIZE       512
const char* g_redirect_fmt = "HTTP/1.0 302 Moved Temporarily\r\n"   \
                             "Location: http://%s\r\n" \
                             "Content-Type: text/html; \r\n" \
                             "Content-Length: 0\r\n\r\n";


nf_hookfn bwlist_post_v4;
nf_hookfn bwlist_post_v6;

static struct nf_hook_ops bwlist_hook_ops[] = {
    {
        .hook = bwlist_post_v4,
        .owner = THIS_MODULE,
        .pf = NFPROTO_IPV4,
        .hooknum = NF_INET_POST_ROUTING,
        .priority = NF_IP_PRI_LAST,
    },
    {
        .hook = bwlist_post_v6,
        .owner = THIS_MODULE,
        .pf = NFPROTO_IPV6,
        .hooknum = NF_INET_POST_ROUTING,
        .priority = NF_IP_PRI_LAST,
    },
};

typedef struct bwlist_exclusive {
    struct bwlist_watch_dev         *watch_devs;
    struct bwlist_exclusive_proto   *protos;
} bwlist_exclusive_t;

typedef struct bwlist_grp_list {
    struct list_head                list;
    bwlist_group_t                  *grp;
    uint32_t                        grp_alloc_by_vmalloc;
} bwlist_grp_list_t;

struct bwlist_mgr {
    volatile int                    enable;
    volatile uint32_t               type;
    bwlist_exclusive_t              *exclusive;
    struct list_head                list;

    char                            *redirect_data[NR_CPUS];
    char                            default_url[MAX_REDIRECT_URL_LEN];
};



static struct bwlist_mgr g_bwlist_mgr;
static volatile uint32_t g_grp_magic = 1;

uint32_t    g_verbose = 0;

uint32_t bwlist_grp_magic(void)
{
    return g_grp_magic;
}


static inline
int is_bufstr_end_with_NULL(char bufstr[], int size)
{
    if(bufstr[size - 1] != '\0') {
        bufstr[size - 1] = '\0';
        if(strlen(bufstr) == size - 1) {
            return false;
        }
    }
    return true;
}


void bwlist_turn_on(unsigned long arg)
{
    uint32_t type = (uint32_t)arg;

    bwlist_sess_invalid_all();

    g_bwlist_mgr.type = type;
    mb();
    g_bwlist_mgr.enable = 1;

    synchronize_rcu();
    bwlist_debug("enable!\n");
}

void bwlist_turn_off(void)
{
    g_bwlist_mgr.enable = 0;
    synchronize_rcu();
    bwlist_debug("disable!\n");
}


int bwlist_upd_default_url(unsigned long arg)
{
    char url[MAX_REDIRECT_URL_LEN] = {0};
    int offset = 0;

    if(copy_from_user(url, (void*)arg, MAX_REDIRECT_URL_LEN)) {
        bwlist_debug("ioctl update default url failed\n");
        return -EINVAL;
    }

    if(!is_bufstr_end_with_NULL(url, MAX_REDIRECT_URL_LEN)) {
        return -EINVAL;
    }

    if(strncmp(url, HTTP_PREFEX, sizeof(HTTP_PREFEX) - 1) == 0) {
        offset += sizeof(HTTP_PREFEX) - 1;
    }

    local_bh_disable();
    strcpy(g_bwlist_mgr.default_url, url + offset);
    local_bh_enable();
    return 0;
}

int is_bwlist_on(void)
{
    return !!g_bwlist_mgr.enable;
}

void hex_printout(const char *msg, const unsigned char *buf, unsigned int len)
{
    static const char hex_char[] = "0123456789ABCDEF";
    const unsigned char *ptr = (const unsigned char*)buf;
    int i, nbytes, j, nlines;
    char msgbuf[120], *dst;

    nlines = ((len + 0x0f) >> 4);
    bwlist_debug("%s--> addr=%08lx %d bytes\n", msg, (unsigned long)buf, len);

    for (j = 0; j < nlines; j++) {
        nbytes = (len < 16 ? len : 16);

        dst = msgbuf;
        memset(dst, 0x20, 4);
        dst += 4;
        for (i = 0; i < nbytes; i++) {
            unsigned char ival = *ptr++;
            *dst ++ = hex_char[(ival >> 4) & 0x0F];
            *dst ++ = hex_char[ival & 0x0F];
            *dst ++ = ' ';
        }

        memset(dst, 0x20, 3 * (17 - nbytes));
        dst += 3 * (17 - nbytes);

        ptr -= nbytes;
        for (i = 0; i < nbytes; i++) {
            if (*ptr >= 0x20 && *ptr <= 0x7e && *ptr != '%') {
                *dst = *ptr;
            } else {
                *dst = '.';
            }
            ptr++;
            dst++;
        }
        *dst++ = '\n';
        *dst = 0;
        bwlist_debug("%s", msgbuf);
        len -= nbytes;
    }
}
EXPORT_SYMBOL_GPL(hex_printout);

int bwlist_grplist_add(bwlist_group_t *grp, int alloc_by_vmalloc)
{
    bwlist_grp_list_t *grp_list;

    grp_list = (bwlist_grp_list_t *)kmalloc(sizeof(*grp_list), GFP_KERNEL);
    if(!grp_list) {
        bwlist_debug("malloc for grp list failed\n");
        goto err;
    }
    memset(grp_list, 0, sizeof(*grp_list));

    if(alloc_by_vmalloc) {
        grp_list->grp_alloc_by_vmalloc = 1;
    }
    grp_list->grp = grp;

    local_bh_disable();
    list_add_tail_rcu(&grp_list->list, &g_bwlist_mgr.list);
    g_grp_magic++;
    local_bh_enable();
    return 0;

err:
    if(grp) {
        if(alloc_by_vmalloc) {
            vfree(grp);
        } else {
            kfree(grp);
        }
    }
    return -1;
}

void bwlist_grplist_destroy(void)
{
    bwlist_grp_list_t *pos, *n;

    list_for_each_entry_safe(pos, n, &g_bwlist_mgr.list, list) {
        list_del_init(&pos->list);

        if(pos->grp_alloc_by_vmalloc) {
            vfree(pos->grp);
        } else {
            kfree(pos->grp);
        }
        kfree(pos);
    }
}


int bwlist_add_grp(unsigned long data)
{
    bwlist_group_t bwgrp, *new_bwgrp;
    int alloc_by_vmalloc;
    int size;
    int ret;
    int i, j;

    bwlist_debug("bwlist_update in\n");

    if(copy_from_user(&bwgrp, (void*)data, sizeof(bwgrp))) {
        bwlist_debug("ioctl update bwlist, bad input\n");
        return -EINVAL;
    }
    if(bwgrp.magic != BWLIST_MAGIC) {
        bwlist_debug("ioctl update bwlist, bad magic\n");
        return -EINVAL;
    }

    size = sizeof(bwlist_group_t) + bwgrp.item_cnt * sizeof(bwlist_item_t);
    if(size > MAX_KMALLOC_SIZE) {
        new_bwgrp = (bwlist_group_t*)vmalloc(size);
        alloc_by_vmalloc = 1;
    } else {
        new_bwgrp = (bwlist_group_t*)kmalloc(size, GFP_KERNEL);
        alloc_by_vmalloc = 0;
    }

    if(!new_bwgrp) {
        bwlist_debug("ioctl update bwlist, not enough memory\n");
        return -ENOMEM;
    }
    memset(new_bwgrp, 0, size);

    if(copy_from_user(new_bwgrp, (void*)data, size)) {
        bwlist_debug("ioctl update bwlist, bad input\n");
        ret = -EINVAL;
        goto err;
    }


    if(!is_bufstr_end_with_NULL(new_bwgrp->redirect_url, MAX_REDIRECT_URL_LEN)) {
        ret = -EINVAL;
        goto err;
    }
    if(strncmp(new_bwgrp->redirect_url, HTTP_PREFEX, sizeof(HTTP_PREFEX) - 1) == 0) {
        int url_raw_len = strlen(new_bwgrp->redirect_url) - (sizeof(HTTP_PREFEX) - 1);
        memmove(new_bwgrp->redirect_url, new_bwgrp->redirect_url + (sizeof(HTTP_PREFEX) - 1), url_raw_len);
        new_bwgrp->redirect_url[url_raw_len] = '\0';
    }

    for(i = 0; i < new_bwgrp->item_cnt; i++) {
        if(new_bwgrp->item[i].type == BWLIST_ITEM_TYPE_IPPORT) {
            if(new_bwgrp->item[i].port.tcp_port_num > ARRAY_SIZE(new_bwgrp->item[i].port.tcp_port)) {
                ret = -EINVAL;
                goto err;
            }
            if(new_bwgrp->item[i].port.udp_port_num > ARRAY_SIZE(new_bwgrp->item[i].port.udp_port)) {
                ret = -EINVAL;
                goto err;
            }
        } else if(new_bwgrp->item[i].type == BWLIST_ITEM_TYPE_KEYPORT) {
            if(!is_bufstr_end_with_NULL(new_bwgrp->item[i].key, MAX_KEYWORD_SIZE)) {
                ret = -EINVAL;
                goto err;
            }

            for(j = 0; j < strlen(new_bwgrp->item[i].key); j++) {
                new_bwgrp->item[i].key[j] = tolower(new_bwgrp->item[i].key[j]);
            }
        }
    }

    return bwlist_grplist_add(new_bwgrp, alloc_by_vmalloc);

err:
    if(alloc_by_vmalloc) {
        vfree(new_bwgrp);
    } else {
        kfree(new_bwgrp);
    }
    return ret;
}


void bwlist_clean_grp(void)
{
    local_bh_disable();

    bwlist_dnsIP_invalid_all();
    bwlist_grplist_destroy();

    local_bh_enable();
}

struct bwlist_exclusive *bwlist_exclusive_init(void)
{
    struct bwlist_exclusive *me;
    me = kzalloc(sizeof(*me), GFP_KERNEL);
    if(!me) {
        return NULL;
    }

    /* 必须保证 devs,protos 是有效的 */
    me->watch_devs = (struct bwlist_watch_dev *)kzalloc(sizeof(struct bwlist_watch_dev), GFP_KERNEL);
    me->protos = (struct bwlist_exclusive_proto *)kzalloc(sizeof(struct bwlist_exclusive_proto), GFP_KERNEL);

    if(!me->watch_devs || !me->protos) {
        if(me->watch_devs) {
            kfree(me->watch_devs);
        }
        if(me->protos) {
            kfree(me->protos);
        }
        kfree(me);
        return NULL;
    }


    return me;
}

void bwlist_exclusive_fini(struct bwlist_exclusive *me)
{
    if(me) {
        if(me->watch_devs) {
            kfree(me->watch_devs);
        }
        if(me->protos) {
            kfree(me->protos);
        }
        kfree(me);
        me = NULL;
    }
}

int is_bwlist_exclusive(struct net_device *outdev, uint8_t protocol)
{
    int match_dev_flag = 0;
    int i;

    for(i = 0; i < g_bwlist_mgr.exclusive->watch_devs->count; i++) {
        if(strcmp(outdev->name, g_bwlist_mgr.exclusive->watch_devs->dev[i].name) == 0) {
            match_dev_flag = 1;
            break;
        }
    }

    if(!match_dev_flag) {
        return 1;
    }

    for(i = 0; i < g_bwlist_mgr.exclusive->protos->count; i++) {
        if(protocol == g_bwlist_mgr.exclusive->protos->proto[i]) {
            return 1;
        }
    }
    return 0;
}

int bwlist_upd_watch_dev(unsigned long data)
{
    bwlist_watch_dev_t watch_dev;
    bwlist_watch_dev_t *new_watch_dev, *old_watch_dev;
    int size, ret;
    int i;

    bwlist_debug("bwlist_update_exclusive_dev in\n");

    if(copy_from_user(&watch_dev, (void*)data, sizeof(watch_dev))) {
        bwlist_debug("ioctl update exclusive dev, bad input\n");
        return -EINVAL;
    }
    if(watch_dev.magic != BWLIST_MAGIC) {
        bwlist_debug("ioctl update exclusive dev, bad magic\n");
        return -EINVAL;
    }

    size = sizeof(struct bwlist_watch_dev) + sizeof(watch_dev.dev[0]) * watch_dev.count;
    new_watch_dev = (struct bwlist_watch_dev *)kzalloc(size, GFP_KERNEL);
    if(!new_watch_dev) {
        bwlist_debug("ioctl update exclusive dev, no memory\n");
        return -ENOMEM;
    }

    if(copy_from_user(new_watch_dev, (void*)data, size)) {
        bwlist_debug("ioctl update exclusive dev, bad input\n");
        ret = -EINVAL;
        goto err;
    }

    for(i = 0; i < new_watch_dev->count; i++) {
        if(!is_bufstr_end_with_NULL(new_watch_dev->dev[i].name, sizeof(new_watch_dev->dev[i].name))) {
            bwlist_debug("ioctl update exclusive dev, dev[%d] name bad len\n", i);
            ret = -EINVAL;
            goto err;
        }
    }

    old_watch_dev = g_bwlist_mgr.exclusive->watch_devs;
    wmb();
    g_bwlist_mgr.exclusive->watch_devs = new_watch_dev;
    synchronize_rcu();

    if(old_watch_dev) {
        kfree(old_watch_dev);
    }
    return 0;

err:
    kfree(new_watch_dev);
    return ret;
}

int bwlist_upd_exclusive_proto(unsigned long data)
{
    bwlist_exclusive_proto_t exclusive_proto;
    bwlist_exclusive_proto_t *new_exclusive_proto, *old_exclusive_proto;
    int size, ret;

    bwlist_debug("bwlist_exclusive_upd_proto in\n");

    if(copy_from_user(&exclusive_proto, (void*)data, sizeof(exclusive_proto))) {
        bwlist_debug("ioctl update exclusive protocol, bad input\n");
        return -EINVAL;
    }
    if(exclusive_proto.magic != BWLIST_MAGIC) {
        bwlist_debug("ioctl update exclusive protocol, bad magic\n");
        return -EINVAL;
    }

    size = sizeof(struct bwlist_exclusive_proto) + sizeof(exclusive_proto.proto[0]) * exclusive_proto.count;
    new_exclusive_proto = (struct bwlist_exclusive_proto *)kzalloc(size, GFP_KERNEL);
    if(!new_exclusive_proto) {
        bwlist_debug("ioctl update exclusive protocol, no memory\n");
        return -ENOMEM;
    }

    if(copy_from_user(new_exclusive_proto, (void*)data, size)) {
        bwlist_debug("ioctl update exclusive protocol, bad input\n");
        ret = -EINVAL;
        goto err;
    }

    old_exclusive_proto = g_bwlist_mgr.exclusive->protos;
    wmb();
    g_bwlist_mgr.exclusive->protos = new_exclusive_proto;
    synchronize_rcu();

    if(old_exclusive_proto) {
        kfree(old_exclusive_proto);
    }
    return 0;

err:
    kfree(new_exclusive_proto);
    return ret;
}




int bwlist_exclusive_dump(char *buf, int size)
{
    int len = 0;
    int i;

#define dump_str(fmt...) \
    do { \
        if(!buf) { \
            printk(fmt); \
        } else if(size - len > 0) { \
            len += snprintf(buf + len, size - len, fmt); \
        } \
    }while(0)

    dump_str("################ bwlist watch devs info ################\n");
    dump_str("devname: ");
    for(i = 0; i < g_bwlist_mgr.exclusive->watch_devs->count; i++) {
        dump_str("%s ", g_bwlist_mgr.exclusive->watch_devs->dev[i].name);
    }
    dump_str("\n");
    dump_str("################ bwlist watch devs end  ################\n");

    dump_str("##############   bwlist exclusive info   ###############\n");
    dump_str("protocol: ");
    for(i = 0; i < g_bwlist_mgr.exclusive->protos->count; i++) {
        dump_str("%d ", g_bwlist_mgr.exclusive->protos->proto[i]);
    }
    dump_str("\n");
    dump_str("############## bwlist exclusive info end ###############\n");

    return len;
#undef dump_str
}

int bwlist_dump(char *buf, int size)
{
    bwlist_grp_list_t *pos;
    int len = 0;
    int grp_idx = 0;
    int i, j;

#define dump_str(fmt...) \
    do { \
        if(!buf) { \
            printk(fmt); \
        } else if(size - len > 0) { \
            len += snprintf(buf + len, size - len, fmt); \
        } \
    }while(0)

    dump_str("##################### bwlist info #####################\n");
    dump_str("active: %s\n", is_bwlist_on() == 1 ? "true" : "false");
    dump_str("type: %s\n", g_bwlist_mgr.type == BWLIST_TYPE_WHITE_LIST ? "whitelist" : "blacklist");
    dump_str("default redirect url: %s\n", g_bwlist_mgr.default_url);


    list_for_each_entry_rcu(pos, &g_bwlist_mgr.list, list) {
        dump_str("group idx: %d, item cnt: %d\n", grp_idx++, pos->grp->item_cnt);
        dump_str("\tredirect url: %s\n", pos->grp->redirect_url);
        for(i = 0; i < pos->grp->item_cnt; i++) {
            bwlist_item_t *item = &pos->grp->item[i];
            bwlist_port_t *port = &item->port;
            if(item->type == BWLIST_ITEM_TYPE_IPPORT) {
                /* ip list */
                for(j = 0; j < MAX_IP_CNT; j++) {
                    if(item->ip[j].ip_base == 0) {
                        break;
                    }
                    dump_str("\tip: %u.%u.%u.%u, mask: %u.%u.%u.%u\n",
                             HIPQUAD(item->ip[j].ip_base),
                             HIPQUAD(item->ip[j].ip_mask));
                }
            } else {
                dump_str("\tkey: %s\n", item->key);
            }


            /* tcp port list */
            dump_str("\ttcp: ");
            if(port->tcp_port_flag) {
                dump_str("%u~%u ", port->tcp_begin_port, port->tcp_end_port);
            }
            for(j = 0; j < port->tcp_port_num; j++) {
                dump_str("%u ", port->tcp_port[j]);
            }
            dump_str("\n");

            /* udp port list */
            dump_str("\tudp: ");
            if(port->udp_port_flag) {
                dump_str("%u~%u ", port->udp_begin_port, port->udp_end_port);
            }
            for(j = 0; j < port->udp_port_num; j++) {
                dump_str("%u ", port->udp_port[j]);
            }
            dump_str("\n");
        }
    }

    dump_str("################### bwlist info end ###################\n");
    return len;
#undef dump_str
}

int is_in_exclusive(struct sk_buff *skb, struct net_device *outdev)
{
    struct iphdr *iph;
    struct udphdr *udph;
    unsigned char *h_raw;

    iph = (struct iphdr *)skb_network_header(skb);
    h_raw = (unsigned char*)iph + (iph->ihl << 2);

    if(is_bwlist_exclusive(outdev, iph->protocol)) {
        return true;
    }

    if(iph->protocol == IPPROTO_UDP) {
        udph = (struct udphdr *)h_raw;
#define DNS_PORT (53)
        if(udph->dest == __constant_htons(DNS_PORT)
                || udph->source == __constant_htons(DNS_PORT)) {
            return true;
        }
    }

    return false;
}

int check_port_range(bwlist_port_t *port, uint16_t h_dstport, uint16_t h_srcport, int is_tcp)
{
    if(is_tcp) {
        if(port->tcp_port_flag) {
            if(h_dstport >= port->tcp_begin_port && h_dstport <= port->tcp_end_port) {
                bwlist_debug("match tcp range ok\n");
                return 1;
            }
            if(h_srcport >= port->tcp_begin_port && h_srcport <= port->tcp_end_port) {
                bwlist_debug("match tcp range ok\n");
                return 1;
            }
        }
    } else {
        if(port->udp_port_flag) {
            if(h_dstport >= port->udp_begin_port && h_dstport <= port->udp_end_port) {
                bwlist_debug("match udp range ok\n");
                return 1;
            }
            if(h_srcport >= port->udp_begin_port && h_srcport <= port->udp_end_port) {
                bwlist_debug("match udp range ok\n");
                return 1;
            }
        }
    }
    return 0;
}

int check_port_single(bwlist_port_t *port, uint16_t h_dstport, uint16_t h_srcport, int is_tcp)
{
    int i;
    if(is_tcp) {
        for(i = 0; i < MAX_PORT_NUM; i++) {
            if(h_dstport == port->tcp_port[i] || h_srcport == port->tcp_port[i]) {
                bwlist_debug("match tcp port ok\n");
                return 1;
            }
        }
    } else {
        for(i = 0; i < MAX_PORT_NUM; i++) {
            if(h_dstport == port->udp_port[i] || h_srcport == port->udp_port[i]) {
                bwlist_debug("match udp port ok\n");
                return 1;
            }
        }
    }
    return 0;
}

inline int is_matched_port(bwlist_port_t *port, uint16_t h_dstport, uint16_t h_srcport, int is_tcp)
{
    bwlist_debug("is_matched_port, h_dstport: %d, h_srcport: %d, istcp: %d\n", h_dstport, h_srcport, is_tcp);

    if(check_port_range(port, h_dstport, h_srcport, is_tcp)) {
        return 1;
    }

    if(check_port_single(port, h_dstport, h_srcport, is_tcp)) {
        return 1;
    }

    bwlist_debug("match failed\n");
    return 0;
}

inline int is_matched_ip(bwlist_item_t *item, uint32_t h_daddr)
{
    int i;
    bwlist_debug("is_matched_ip, h_daddr: %u.%u.%u.%u\n", HIPQUAD(h_daddr));

    /* IP全为空时，默认表示全匹配； */
    if(item->ip[0].ip_mask == 0) {
        bwlist_debug("is_matched_ip, matche all ip\n");
        return 1;
    }

    for(i = 0; i < MAX_IP_CNT; i++) {
        /* 排除末尾未填的情况 */
        if(item->ip[i].ip_base == 0 && item->ip[i].ip_mask == 0) {
            break;
        }
        
        if((item->ip[i].ip_base & item->ip[i].ip_mask) == (h_daddr & item->ip[i].ip_mask)) {
            bwlist_debug("is_matched_ip, matched ok\n");
            return 1;
        }
    }

    bwlist_debug("is_matched_ip, matched failed\n");
    return 0;
}

bwlist_group_t *bwlist_find_grp_with_ipport(struct iphdr *iph)
{
    bwlist_grp_list_t *pos, *n;
    int i;

    list_for_each_entry_safe(pos, n, &g_bwlist_mgr.list, list) {
        for(i = 0; i < pos->grp->item_cnt; i++) {
            bwlist_item_t *item = &pos->grp->item[i];

            if(item->type == BWLIST_ITEM_TYPE_IPPORT && is_matched_ip(item, ntohl(iph->daddr))) {
                uint16_t dstport, srcport;
                unsigned char *h_data = (unsigned char *)iph + (iph->ihl << 2);

                if(iph->protocol == IPPROTO_TCP) {
                    dstport = ntohs(((struct tcphdr *)h_data)->dest);
                    srcport = ntohs(((struct tcphdr *)h_data)->source);
                    if(is_matched_port(&item->port, dstport, srcport, 1)) {
                        return pos->grp;
                    }
                } else if(iph->protocol == IPPROTO_UDP) {
                    dstport = ntohs(((struct udphdr *)h_data)->dest);
                    srcport = ntohs(((struct udphdr *)h_data)->source);
                    if(is_matched_port(&item->port, dstport, srcport, 0)) {
                        return pos->grp;
                    }
                } else {
                    bwlist_debug("not matched(protocol undefined)..\n");
                    return NULL;
                }
            }
        }
    }
    bwlist_debug("not matched..\n");
    return NULL;
}

bwlist_group_t *bwlist_find_grp_with_key(const unsigned char *domain)
{
    bwlist_grp_list_t *pos, *n;
    int i;

    list_for_each_entry_safe(pos, n, &g_bwlist_mgr.list, list) {
        for(i = 0; i < pos->grp->item_cnt; i++) {
            bwlist_item_t *item = &pos->grp->item[i];

            if(item->type == BWLIST_ITEM_TYPE_KEYPORT && strstr(domain, item->key)) {
                bwlist_debug("find key ok, domain: %s, key: %s\n", domain, item->key);
                return pos->grp;
            }
        }
    }
    bwlist_debug("find key failed, domain: %s\n", domain);
    return NULL;
}

int do_redirect(struct sk_buff *ori_skb, const struct net_device *outdev, char *data, int datalen)
{
    struct sk_buff *skb;
    int len;
    struct iphdr *iph, *old_iph;
    struct tcphdr *tcph, *old_tcph;
    int rc;
    char fak_mac[ETH_ALEN] = {0x0, 0x0, 0x0, 0x0, 0x01, 0x01};
    struct ethhdr *old_mac;


    old_mac = (struct ethhdr *)(ori_skb->data - ETH_HLEN);
    bwlist_debug("ori_skb mac_h: %p, data: %p\n", old_mac, ori_skb->data);
    //hex_printout("orig data", ori_skb->data - ETH_HLEN, ori_skb->len + ETH_HLEN);

    old_iph = (struct iphdr *)skb_network_header(ori_skb);
    old_tcph = (struct tcphdr *)((char *)old_iph + (old_iph->ihl << 2));

    len = datalen + ETH_HLEN + sizeof(struct iphdr) + sizeof(struct tcphdr);
    skb = dev_alloc_skb(len);
    if (!skb) {
        bwlist_debug("alloc skb failed\n");
        return -ENOMEM;
    }

    skb_put(skb, len);
    skb_reset_mac_header(skb);

    memcpy(eth_hdr(skb)->h_dest, outdev->dev_addr, ETH_ALEN);
    memcpy(eth_hdr(skb)->h_source, fak_mac, ETH_ALEN);
    eth_hdr(skb)->h_proto = __constant_htons(ETH_P_IP);

    skb->dev = (struct net_device *)outdev;


    skb_set_network_header(skb, ETH_HLEN);
    iph = (struct iphdr *)skb_network_header(skb);

    //填充IP头
    iph->version = 4;
    iph->ihl = sizeof(struct iphdr) >> 2;
    iph->tos = 0;
    iph->tot_len = htons((iph->ihl << 2) + sizeof(struct tcphdr) + datalen);
    iph->id = 0x2658;
    iph->frag_off = 0;
    iph->ttl = 64;
    iph->protocol = IPPROTO_TCP;
    iph->daddr = old_iph->saddr;
    iph->saddr = old_iph->daddr;
    ip_send_check(iph);

    // 填充TCP头
    skb_set_transport_header(skb, ETH_HLEN + (iph->ihl << 2));
    tcph = (struct tcphdr *)skb_transport_header(skb);
    memset(tcph, 0, sizeof(struct tcphdr));

    tcph->source = old_tcph->dest;
    tcph->dest = old_tcph->source;
    tcph->seq = old_tcph->ack_seq;
    len = ntohs(old_iph->tot_len) - (old_iph->ihl << 2) - (old_tcph->doff << 2);
    tcph->ack_seq = htonl(ntohl(old_tcph->seq) + len);
    tcph->doff = sizeof(struct tcphdr) >> 2;
    tcph->ack = 1;
    tcph->psh = 1;
    tcph->fin = 1;
    tcph->rst = 0;
    tcph->window = 65535;
    tcph->check = 0;

    memcpy((char*)tcph + (tcph->doff << 2), data, datalen);

    skb->csum = csum_partial(tcph, ntohs(iph->tot_len) - (iph->ihl << 2), 0);

    tcph->check = tcp_v4_check(ntohs(iph->tot_len) - (iph->ihl << 2),
                               iph->saddr, iph->daddr, skb->csum);

    skb->ip_summed = CHECKSUM_NONE;

    //hex_printout("do_redirect", skb_mac_header(skb), skb->len);
    bwlist_debug("do_redirect, new skb: %p, macp: %p, iph: %p, tcph: %p\n",
                 skb, eth_hdr(skb), iph, tcp_hdr(skb));

    skb->protocol = eth_type_trans(skb, skb->dev);
    rc = netif_rx(skb);

    return rc;
}

int send_reset_to_reply_dir(struct sk_buff *ori_skb, const struct net_device *outdev)
{
    struct sk_buff *skb;
    int len;
    struct iphdr *iph, *old_iph;
    struct tcphdr *tcph, *old_tcph;
    int rc;
    char fak_mac[ETH_ALEN] = {0x0, 0x0, 0x0, 0x0, 0x01, 0x02};
    struct ethhdr *old_mac;


    old_mac = (struct ethhdr *)(ori_skb->data - ETH_HLEN);
    bwlist_debug("ori_skb mac_h: %p, data: %p\n", old_mac, ori_skb->data);
    //hex_printout("orig data", ori_skb->data - ETH_HLEN, ori_skb->len + ETH_HLEN);

    old_iph = (struct iphdr *)skb_network_header(ori_skb);
    old_tcph = (struct tcphdr *)((char *)old_iph + (old_iph->ihl << 2));

    len = ETH_HLEN + sizeof(struct iphdr) + sizeof(struct tcphdr);
    skb = dev_alloc_skb(len);
    if (!skb) {
        bwlist_debug("alloc skb failed\n");
        return -ENOMEM;
    }

    skb_put(skb, len);
    skb_reset_mac_header(skb);

    memcpy(eth_hdr(skb)->h_dest, outdev->dev_addr, ETH_ALEN);
    memcpy(eth_hdr(skb)->h_source, fak_mac, ETH_ALEN);
    eth_hdr(skb)->h_proto = __constant_htons(ETH_P_IP);

    skb->dev = (struct net_device *)outdev;


    skb_set_network_header(skb, ETH_HLEN);
    iph = (struct iphdr *)skb_network_header(skb);

    //填充IP头
    iph->version = 4;
    iph->ihl = sizeof(struct iphdr) >> 2;
    iph->tos = 0;
    iph->tot_len = __constant_htons((iph->ihl << 2) + sizeof(struct tcphdr));
    iph->id = 0x2658;
    iph->frag_off = 0;
    iph->ttl = 64;
    iph->protocol = IPPROTO_TCP;
    iph->daddr = old_iph->saddr;
    iph->saddr = old_iph->daddr;
    ip_send_check(iph);

    // 填充TCP头
    skb_set_transport_header(skb, ETH_HLEN + (iph->ihl << 2));
    tcph = (struct tcphdr *)skb_transport_header(skb);
    memset(tcph, 0, sizeof(struct tcphdr));

    tcph->source = old_tcph->dest;
    tcph->dest = old_tcph->source;
    tcph->seq = old_tcph->ack_seq;
    len = __constant_ntohs(old_iph->tot_len) - (old_iph->ihl << 2) - (old_tcph->doff << 2);
    tcph->ack_seq = __constant_htonl(__constant_ntohl(old_tcph->seq) + len);
    tcph->doff = sizeof(struct tcphdr) >> 2;
    tcph->ack = 0;
    tcph->psh = 0;
    tcph->fin = 0;
    tcph->rst = 1;
    tcph->window = 65535;
    tcph->check = 0;

    skb->csum = csum_partial(tcph, __constant_ntohs(iph->tot_len) - (iph->ihl << 2), 0);

    tcph->check = tcp_v4_check(__constant_ntohs(iph->tot_len) - (iph->ihl << 2),
                               iph->saddr, iph->daddr, skb->csum);

    skb->ip_summed = CHECKSUM_NONE;

    //hex_printout("send_reset_to_reply_dir", skb_mac_header(skb), skb->len);

    skb->protocol = eth_type_trans(skb, skb->dev);
    rc = netif_rx(skb);

    return rc;
}

void send_reset_to_forward_dir(struct sk_buff *ori_skb)
{
    struct sk_buff *skb;
    int len;
    struct ethhdr *old_mac;
    struct iphdr *iph, *old_iph;
    struct tcphdr *tcph, *old_tcph;
    char fak_mac[ETH_ALEN] = {0x0, 0x0, 0x0, 0x0, 0x02, 0x02};

    old_iph = (struct iphdr *)skb_network_header(ori_skb);
    old_tcph = (struct tcphdr *)((char *)old_iph + (old_iph->ihl << 2));

    len = ETH_HLEN + old_iph->ihl * 4 + sizeof(struct tcphdr);
    skb = dev_alloc_skb(len);
    if (!skb) {
        bwlist_debug("alloc skb failed\n");
        return ;
    }
    skb_put(skb, len);
    skb_reset_mac_header(skb);

    old_mac = (struct ethhdr *)(ori_skb->data - ETH_HLEN);
    memcpy(eth_hdr(skb)->h_dest, old_mac->h_dest, ETH_ALEN);
    memcpy(eth_hdr(skb)->h_source, fak_mac, ETH_ALEN);
    eth_hdr(skb)->h_proto = __constant_htons(ETH_P_IP);

    skb->dev = ori_skb->dev;

    skb_set_network_header(skb, ETH_HLEN);
    iph = (struct iphdr *)skb_network_header(skb);

    //填充IP头
    memcpy(iph, old_iph, old_iph->ihl << 2);
    iph->tot_len = __constant_htons((iph->ihl << 2) + sizeof(struct tcphdr));
    iph->check = 0;
    ip_send_check(iph);

    // 填充TCP头
    skb_set_transport_header(skb, ETH_HLEN + (iph->ihl << 2));
    tcph = (struct tcphdr *)skb_transport_header(skb);
    memset(tcph, 0, sizeof(struct tcphdr));

    tcph->source = old_tcph->source;
    tcph->dest = old_tcph->dest;
    tcph->seq = old_tcph->seq;
    tcph->ack_seq = old_tcph->ack_seq;
    tcph->doff = sizeof(struct tcphdr) >> 2;
    tcph->ack = 0;
    tcph->psh = 0;
    tcph->fin = 0;
    tcph->rst = 1;
    tcph->window = 65535;
    tcph->check = 0;

    skb->csum = csum_partial(tcph, __constant_ntohs(iph->tot_len) - (iph->ihl << 2), 0);
    tcph->check = tcp_v4_check(__constant_ntohs(iph->tot_len) - (iph->ihl << 2),
                               iph->saddr, iph->daddr, skb->csum);

    skb->ip_summed = CHECKSUM_NONE;

    //hex_printout("send_reset_to_forward_dir", skb_mac_header(skb), skb->len);

    dev_queue_xmit(skb);
    return ;
}

int redirect_to_url(struct sk_buff *skb, struct net_device *outdev, bwlist_group_t *grp)
{
    int datalen;
    int cpu;
    char *redirect_url;

    if(grp && strlen(grp->redirect_url) > 0) {
        redirect_url = grp->redirect_url;
    } else {
        redirect_url = g_bwlist_mgr.default_url;
    }

    if(strlen(redirect_url) > 0) {
        cpu = get_cpu();
        datalen = snprintf(g_bwlist_mgr.redirect_data[cpu], MAX_DATA_SIZE - 1, g_redirect_fmt, redirect_url);
        do_redirect(skb, outdev, g_bwlist_mgr.redirect_data[cpu], datalen);
        put_cpu();
    } else {
        send_reset_to_reply_dir(skb, outdev);
    }

    send_reset_to_forward_dir(skb);
    return 0;
}


unsigned int bwlist_do_filter(struct sk_buff *skb, struct net_device *outdev)
{
    bwlist_group_t *grp;
    struct iphdr *iph;
    unsigned char *h_data;
    struct tcphdr *tcph;
    struct udphdr *udph;
    unsigned char *data;
    int datalen;

    uint32_t lan_ip, wan_ip;
    uint16_t lan_port, wan_port;
    uint8_t protocol;
    int action;


    if(!is_bwlist_on()) {
        return NF_ACCEPT;
    }

    if(is_in_exclusive(skb, outdev)) {
        return NF_ACCEPT;
    }

    iph = (struct iphdr *)skb_network_header(skb);
    h_data = (unsigned char *)iph + (iph->ihl << 2);


    lan_ip = iph->saddr;
    wan_ip = iph->daddr;
    protocol = iph->protocol;
    if(iph->protocol == IPPROTO_TCP) {
        tcph = (struct tcphdr *)h_data;
        lan_port = tcph->source;
        wan_port = tcph->dest;
    } else if(iph->protocol == IPPROTO_UDP) {
        udph = (struct udphdr *)h_data;
        lan_port = udph->source;
        wan_port = udph->dest;
    } else {
        lan_port = 0;
        wan_port = 0;
    }


    action = bwlist_sess_fresh(lan_ip, wan_ip, lan_port, wan_port, protocol);
    if(action == BWLIST_SESS_ACCEPT) {
        return NF_ACCEPT;
    } else if(action == BWLIST_SESS_DROP) {
        /* 防止两端不停地重发fin包 */
        if(iph->protocol == IPPROTO_TCP) {
            tcph = (struct tcphdr *)h_data;
            if(tcph->rst /* || tcph->fin*/ ) {
                return NF_ACCEPT;
            }
            send_reset_to_reply_dir(skb, outdev);
        }

        bwlist_debug("11 drop, %u.%u.%u.%u(%u) -> %u.%u.%u.%u(%u), proto: %d\n",
                     NIPQUAD(lan_ip), ntohs(lan_port),
                     NIPQUAD(wan_ip), ntohs(wan_port),
                     protocol);
        return NF_DROP;
    } else {
        /* BWLIST_SESS_DETECTING */
    }

    rcu_read_lock();
    grp = __bwlist_find_grp_with_dnsIP(ntohl(iph->daddr));
    if(!grp) {
        grp = bwlist_find_grp_with_ipport(iph);
    }

    if(g_bwlist_mgr.type == BWLIST_TYPE_BLACK_LIST && !grp) {
        goto accept_sess;
    }
    if(g_bwlist_mgr.type == BWLIST_TYPE_WHITE_LIST && grp) {
        goto accept_sess;
    }

    if(protocol == IPPROTO_TCP) {
        tcph = (struct tcphdr *)((unsigned char *)iph + (iph->ihl << 2));
        data = (char *)tcph + (tcph->doff << 2);
        datalen = ntohs(iph->tot_len) - (iph->ihl << 2) - (tcph->doff << 2);
        if(datalen == 0) {
            rcu_read_unlock();
            return NF_ACCEPT;
        }

        if(datalen > 4 && strncmp(data, "GET ", sizeof("GET ") - 1) == 0) {
            redirect_to_url(skb, outdev, grp);
        }
    }

    rcu_read_unlock();
    bwlist_sess_set_result(lan_ip, wan_ip, lan_port, wan_port, protocol, BWLIST_SESS_DROP);
#if 1
    /* 不能换成nf_drop, 否则本机重定向有问题，原因待查 */
    kfree_skb(skb);
    return NF_STOLEN;
#else
    return NF_DROP;
#endif

accept_sess:
    rcu_read_unlock();
    bwlist_sess_set_result(lan_ip, wan_ip, lan_port, wan_port, protocol, BWLIST_SESS_ACCEPT);
    return NF_ACCEPT;
}

unsigned int bwlist_post_v4(
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
    return bwlist_do_filter(skb, (struct net_device *)out);
}


unsigned int bwlist_post_v6(
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



int bwlist_mgr_init(void)
{
    int i;
    memset(&g_bwlist_mgr, 0, sizeof(g_bwlist_mgr));

    INIT_LIST_HEAD(&g_bwlist_mgr.list);

    g_bwlist_mgr.exclusive = bwlist_exclusive_init();
    if(!g_bwlist_mgr.exclusive) {
        bwlist_debug("exclusive init failed\n");
        return -1;
    }

    for_each_possible_cpu(i) {
        g_bwlist_mgr.redirect_data[i] = kmalloc(MAX_DATA_SIZE, GFP_KERNEL);
        if(!g_bwlist_mgr.redirect_data[i]) {
            bwlist_debug("malloc for redirect_data failed\n");
            goto err;
        }
        memset(g_bwlist_mgr.redirect_data[i], 0, MAX_DATA_SIZE);
    }
    return 0;

err:
    for_each_possible_cpu(i) {
        if(g_bwlist_mgr.redirect_data[i]) {
            kfree(g_bwlist_mgr.redirect_data[i]);
            g_bwlist_mgr.redirect_data[i] = NULL;
        }
    }

    bwlist_exclusive_fini(g_bwlist_mgr.exclusive);
    return -1;
}

void bwlist_mgr_fini(void)
{
    int i;

    for_each_possible_cpu(i) {
        if(g_bwlist_mgr.redirect_data[i]) {
            kfree(g_bwlist_mgr.redirect_data[i]);
            g_bwlist_mgr.redirect_data[i] = NULL;
        }
    }

    if(g_bwlist_mgr.exclusive) {
        bwlist_exclusive_fini(g_bwlist_mgr.exclusive);
    }

    bwlist_grplist_destroy();
}

static int __init bwlist_init(void)
{
    if(bwlist_mgr_init() < 0) {
        bwlist_debug("bwlist mgr init failed\n");
        return -1;
    }

    if(bwlist_sess_init(2048) < 0) {
        bwlist_debug("sess_init failed\n");
        goto sess_err;
    }

    if(bwlist_dnsIP_init(4096) < 0) {
        bwlist_debug("bwlist dns init failed\n");
        goto dns_err;
    }

    if(nf_register_hooks(bwlist_hook_ops, ARRAY_SIZE(bwlist_hook_ops)) < 0) {
        bwlist_debug("hook register failed\n");
        goto hook_err;
    }

    if(bwlist_interface_init() < 0) {
        bwlist_debug("interface init failed\n");
        goto interface_err;
    }

    bwlist_debug("bwlist_init over\n");
    printk(KERN_EMERG "# bwlist # init over, timestamp: %lu\n", jiffies);

    return 0;

interface_err:
    nf_unregister_hooks(bwlist_hook_ops, ARRAY_SIZE(bwlist_hook_ops));
hook_err:
    bwlist_dnsIP_fini();
dns_err:
    bwlist_sess_fini();
sess_err:
    bwlist_mgr_fini();

    return -1;
}


static void __exit bwlist_exit(void)
{
    bwlist_interface_fini();

    nf_unregister_hooks(bwlist_hook_ops, ARRAY_SIZE(bwlist_hook_ops));

    bwlist_dnsIP_fini();

    bwlist_sess_fini();

    bwlist_mgr_fini();

    bwlist_debug("bwlist_exit over\n");
    printk(KERN_EMERG "# bwlist # exit over, timestamp: %lu\n", jiffies);
}

module_init(bwlist_init);
module_exit(bwlist_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jinsheng@skyroam.com");

