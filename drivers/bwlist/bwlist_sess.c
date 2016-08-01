#include <linux/jhash.h>
#include <linux/jiffies.h>
#include <linux/rcupdate.h>

#include "hashtb_comm.h"
#include "bwlist_sess.h"
#include "bwlist_comm.h"



struct bwlist_sess {
    uint32_t    lan_ip;
    uint32_t    wan_ip;
    uint16_t    lan_port;
    uint16_t    wan_port;
    uint8_t     protocol;

    uint8_t     result;
    unsigned long    fresh_time;
};


#define SESS_NODE_TIMEOUT           (3 * 60 * HZ)
#define SESS_NODE_UPD_DELAY         SESS_NODE_TIMEOUT / 10
#define SESS_NODE_CHECKTIME         (HZ)


static hashtb_t *g_sess_hashtb = NULL;


static void bwlist_sess_node_cleanup(void *node_data)
{
    struct bwlist_sess *node = (struct bwlist_sess *)node_data;
    bwlist_debug("close sess, jiffies: %lu, %u.%u.%u.%u(%u) -> %u.%u.%u.%u(%u)\n",
                 jiffies,
                 NIPQUAD(node->lan_ip), node->lan_port,
                 NIPQUAD(node->wan_ip), node->wan_port);
}

static int bwlist_sess_node_key_match(void *node_data1, void *node_data2)
{
    struct bwlist_sess *node1, *node2;

    node1 = (struct bwlist_sess *)node_data1;
    node2 = (struct bwlist_sess *)node_data2;

    if(node1->lan_ip == node2->lan_ip
            && node1->wan_ip == node2->wan_ip
            && node1->lan_port == node2->lan_port
            && node1->wan_port == node2->wan_port
            && node1->protocol == node2->protocol) {
        return 1;
    }
    return 0;
}

static int bwlist_sess_node_value_match(void *node_data1, void *node_data2)
{
    struct bwlist_sess *node1, *node2;

    node1 = (struct bwlist_sess *)node_data1;
    node2 = (struct bwlist_sess *)node_data2;

    return 1;
}

static uint32_t bwlist_sess_node_hash_key(void *node_data, uint32_t size)
{
    uint32_t hashcode;
    struct bwlist_sess *node = (struct bwlist_sess *)node_data;
    hashcode = jhash_3words(node->lan_ip, node->wan_ip, (node->lan_port << 16) | node->wan_port, 0);
    return hashcode;
}

static int bwlist_sess_node_timeout_check(void *node_data)
{
    struct bwlist_sess *node = (struct bwlist_sess *)node_data;
    if(time_before(jiffies, node->fresh_time + SESS_NODE_TIMEOUT)) {
        return 0;
    }
    return 1;
}

static void bwlist_dump_node(void *node_data)
{
    bwlist_sess_t *sess = (bwlist_sess_t *)node_data;

    bwlist_debug("bwlist sess: %u.%u.%u.%u(%u) -> %u.%u.%u.%u(%u), protocol: %d, timestamp: %lu, result: %s\n",
                 NIPQUAD(sess->lan_ip), ntohs(sess->lan_port),
                 NIPQUAD(sess->wan_ip), ntohs(sess->wan_port),
                 sess->protocol, sess->fresh_time,
                 sess->result == BWLIST_SESS_ACCEPT ? "ACCEPT" : "DROP");
}


int bwlist_sess_set_result(uint32_t lan_ip, uint32_t wan_ip, uint16_t lan_port,
                           uint16_t wan_port, uint8_t protocol, uint8_t result)
{
    int ret;
    struct bwlist_sess *node = ht_data_create(g_sess_hashtb);
    if(!node) {
        return -1;
    }
    node->lan_ip = lan_ip;
    node->wan_ip = wan_ip;
    node->lan_port = lan_port;
    node->wan_port = wan_port;
    node->protocol = protocol;
    node->result = result;
    node->fresh_time = jiffies;

    rcu_read_lock();
    ret = ht_node_update(g_sess_hashtb, node, sizeof(*node));
    rcu_read_unlock();

    if(ret < 0) {
        bwlist_debug("ht_node_update error, errno is %d\n", ret);
        ht_data_destroy(g_sess_hashtb, node, sizeof(*node));
    }
    return ret;
}

int bwlist_sess_fresh(uint32_t lan_ip, uint32_t wan_ip, uint16_t lan_port,
                      uint16_t wan_port, uint8_t protocol)
{
    bwlist_sess_t *sess;
    int ret;
    struct bwlist_sess node = {
        .lan_ip = lan_ip,
        .wan_ip = wan_ip,
        .lan_port = lan_port,
        .wan_port = wan_port,
        .protocol = protocol,
    };

    rcu_read_lock();
    sess = (struct bwlist_sess *)ht_node_find(g_sess_hashtb, &node, sizeof(node));
    if(sess) {
        ret = sess->result;
        
        if(time_after(jiffies, sess->fresh_time + SESS_NODE_UPD_DELAY)) {
            int upd_err;
            upd_err = bwlist_sess_set_result(sess->lan_ip, sess->wan_ip, sess->lan_port, sess->wan_port,
                                             sess->protocol, sess->result);
            if(upd_err < 0) {
                bwlist_debug("update sess failed\n");
            }
        }
    } else {
        ret = BWLIST_SESS_DETECTING;
    }
    rcu_read_unlock();

    return ret;
}

void bwlist_sess_invalid_all(void)
{
    int cnt = ht_cleanup_nodes(g_sess_hashtb);
    bwlist_debug("invalid sess node cnt: %d\n", cnt);
}

void bwlist_sess_dump(void)
{
    int cnt = ht_dump_all(g_sess_hashtb);
    bwlist_debug("dump sess node cnt: %d\n", cnt);
}

int bwlist_sess_node_cnt(void)
{
    return ht_node_cnt(g_sess_hashtb);
}

int bwlist_sess_init(uint32_t max_sess_cnt)
{
    hashtb_t *me;
    me = ht_create("sess", max_sess_cnt, sizeof(struct bwlist_sess),
                   SESS_NODE_CHECKTIME, max_sess_cnt / 60,
                   bwlist_sess_node_key_match, bwlist_sess_node_value_match,
                   bwlist_sess_node_hash_key, bwlist_sess_node_timeout_check,
                   bwlist_sess_node_cleanup, bwlist_dump_node);
    if(!me) {
        return -1;
    }

    g_sess_hashtb = me;
    return 0;
}

void bwlist_sess_fini(void)
{
    if(g_sess_hashtb) {
        ht_destroy(g_sess_hashtb);
        g_sess_hashtb = NULL;
    }
}

