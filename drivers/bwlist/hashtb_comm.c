#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <asm/atomic.h>
#include <linux/rcupdate.h>
#include <linux/vmalloc.h>
#include <linux/rculist.h>
#include <linux/jhash.h>
#include <linux/random.h>

#include "hashtb_comm.h"
#include "bwlist_comm.h"


#define ht_debug bwlist_debug
#define ht_error bwlist_debug


struct hl_head {
    struct hlist_head   hlist;
    spinlock_t          lock;
};

struct hashtb {
    struct hl_head      *head;
    uint32_t            slot_mask;
    uint32_t            hash_salt;
    atomic_t            node_cnt;
    atomic_t            ext_cnt;
    uint16_t            node_data_size;
    uint8_t             alloc_by_vmalloc;
    uint8_t             flags;

    struct kmem_cache   *node_hcache;
    struct kmem_cache   *data_hcache;

    char                *name;

    struct timer_list   timer;
    uint32_t            timer_walk_slot;
    uint32_t            check_interval;     // jiffies
    uint32_t            check_cnt;

    fn_node_key_match   node_key_match;     // need
    fn_node_value_match node_value_match;   // need
    fn_node_hash_key    node_hash_key;      // need
    fn_node_timeout_check node_timeout_check; // need
    fn_node_cleanup     node_cleanup;       // option
    fn_node_dump        node_dump;      //option
};

#define HT_EXT_MAGIC 0x20abcd15
typedef struct ht_ext {
    uint32_t            magic;
    char                data[0];
} ht_ext_t;


struct ht_node {
    struct hlist_node   hlist;
    struct rcu_head     rcu;
    struct hashtb       *parent;

    ht_ext_t           *extend;
};



inline uint32_t ht_node_hash(struct hashtb *tb, void *data, uint32_t size)
{
    return jhash_1word(tb->node_hash_key(data, size), tb->hash_salt) & tb->slot_mask;
}

inline int ht_node_value_match(struct hashtb *tb, void *data1, void *data2)
{
    return tb->node_value_match(data1, data2);
}


inline int ht_node_match(struct hashtb *tb, void *data1, void *data2)
{
    return tb->node_key_match(data1, data2);
}

/**
   返回0表示未超时，返回1表示已超时。
*/
inline int ht_node_timeout_check(struct hashtb *tb, void *data)
{
    return !!tb->node_timeout_check(data);
}

static inline
ht_ext_t *check_ht_ext_ok(struct hashtb *tb, void *data, uint32_t size)
{
    ht_ext_t *ext;

    if(size != tb->node_data_size) {
        bwlist_debug("error!! node size NOT matched\n");
        BUG();
        return NULL;
    }

    ext = container_of(data, ht_ext_t, data);
    if(ext->magic != HT_EXT_MAGIC) {
        bwlist_debug("error!! NOT a ht_data\n");
        BUG();
        return NULL;
    }
    return ext;
}

void *ht_data_create(hashtb_t *tb)
{
    ht_ext_t *extend;
    if(!tb) {
        return NULL;
    }
    extend = kmem_cache_alloc(tb->data_hcache, GFP_ATOMIC);
    if(!extend) {
        return NULL;
    }
    extend->magic = HT_EXT_MAGIC;
    atomic_inc(&tb->ext_cnt);
    return extend->data;
}

void ht_data_destroy(hashtb_t *tb, void *data, uint32_t size)
{
    ht_ext_t *ext;

    if(!tb || !data) {
        return ;
    }
    ext = check_ht_ext_ok(tb, data, size);
    if(ext) {
        atomic_dec(&tb->ext_cnt);
        kmem_cache_free(tb->data_hcache, ext);
    }
}


static void ht_node_destroy(struct ht_node *me)
{
    if(me->parent->node_cleanup) {
        me->parent->node_cleanup(me->extend->data);
    }

    ht_data_destroy(me->parent, me->extend->data, me->parent->node_data_size);

    atomic_dec(&me->parent->node_cnt);
    ht_debug("ht `%s` node destroy, node cnt: %d, ext cnt: %d\n", me->parent->name,
             atomic_read(&me->parent->node_cnt), atomic_read(&me->parent->ext_cnt));
    kmem_cache_free(me->parent->node_hcache, me);
}

static inline int ht_node_preinit(struct hashtb *tb, struct ht_node *node, ht_ext_t *ext)
{
    memset(node, 0, sizeof(*node));
    node->parent = tb;
    INIT_HLIST_NODE(&node->hlist);
    rcu_assign_pointer(node->extend, ext);
    return 0;
}


static struct ht_node *ht_node_create(struct hashtb *tb, ht_ext_t *ext) {
    struct ht_node *me;
    me = kmem_cache_alloc(tb->node_hcache, GFP_ATOMIC);
    if(!me) {
        return NULL;
    }

    if(ht_node_preinit(tb, me, ext) < 0) {
        kmem_cache_free(tb->node_hcache, me);
        return NULL;
    }

    atomic_inc(&tb->node_cnt);
    ht_debug("ht `%s` node create, node: %d, ext: %d\n", tb->name, atomic_read(&tb->node_cnt), atomic_read(&tb->ext_cnt));
    return me;
}

static void ht_rcu_cb(struct rcu_head *rcu)
{
    struct ht_node *ent = container_of(rcu, struct ht_node, rcu);
    ht_node_destroy(ent);
}

static void ht_timer(unsigned long data)
{
    struct hashtb *tb = (struct hashtb*)data;
    struct hlist_node *n;
    struct ht_node *tpos;
    int32_t cnt = 0;
    int ret;
    int i;

    i = tb->timer_walk_slot & tb->slot_mask;
    for(; i < tb->slot_mask + 1; i++) {
        if(cnt >= tb->check_cnt) {
            break;
        }

        hlist_for_each_entry_safe(tpos, n, &tb->head[i].hlist, hlist) {
            cnt++;

            ret = ht_node_timeout_check(tb, tpos->extend->data);
            if(ret) {
                hlist_del_rcu(&tpos->hlist);
                call_rcu(&tpos->rcu, ht_rcu_cb);
            }
        }
    }
    tb->timer_walk_slot = i;

    //ht_debug("ht `%s` node cnt: %d, ext cnt: %d\n", tb->name, atomic_read(&tb->node_cnt), atomic_read(&tb->ext_cnt));
    mod_timer(&tb->timer, jiffies + tb->check_interval);
}


static inline
struct ht_node *__ht_node_find(struct hashtb *tb, uint32_t hash, void *data) {
    struct ht_node *tpos;

    hlist_for_each_entry(tpos, &tb->head[hash].hlist, hlist) {
        if(ht_node_match(tb, tpos->extend->data, data)) {
            return tpos;
        }
    }
    return NULL;
}

void *ht_node_find(struct hashtb *tb, void *data, uint32_t size)
{
    struct ht_node *entry;
    int hash;

    hash = ht_node_hash(tb, data, size);
    entry = __ht_node_find(tb, hash, data);
    if(entry) {
        return entry->extend->data;
    } else {
        return NULL;
    }
}

int ht_node_update(hashtb_t *tb, void *data, uint32_t size)
{
    struct ht_node *new_entry, *entry;
    int hash;
    ht_ext_t *ext;

    ext = check_ht_ext_ok(tb, data, size);
    if(!ext) {
        /* 由外部销毁data */
        return -1;
    }

    new_entry = ht_node_create(tb, ext);
    if(!new_entry) {
        /* 由外部销毁data */
        return -ENOMEM;
    }

    hash = ht_node_hash(tb, data, size);
    spin_lock(&tb->head[hash].lock);
    entry = __ht_node_find(tb, hash, data);
    if(!entry) {
        hlist_add_head_rcu(&new_entry->hlist, &tb->head[hash].hlist);
        spin_unlock(&tb->head[hash].lock);
    } else {
        hlist_del_rcu(&entry->hlist);
        hlist_add_head_rcu(&new_entry->hlist, &tb->head[hash].hlist);

        spin_unlock(&tb->head[hash].lock);

        call_rcu(&entry->rcu, ht_rcu_cb);
    }
    return 0;
}


static int ht_headpool_create(struct hashtb *tb, uint32_t slot_num)
{
    uint32_t size = sizeof(*tb->head) * slot_num;
    int i;

    if(size <= MAX_KMALLOC_SIZE) {
        tb->head = kmalloc(size, GFP_KERNEL);
        tb->alloc_by_vmalloc = 0;
    } else {
        tb->head = vmalloc(size);
        tb->alloc_by_vmalloc = 1;
    }

    if(!tb->head) {
        return -1;
    }

    memset(tb->head, 0, size);
    for(i = 0; i < slot_num; i++) {
        INIT_HLIST_HEAD(&tb->head[i].hlist);
        spin_lock_init(&tb->head[i].lock);
    }
    return 0;
}

static void ht_headpool_destroy(struct hashtb *tb)
{
    if(tb->alloc_by_vmalloc) {
        vfree(tb->head);
    } else {
        kfree(tb->head);
    }
}

int ht_dump_all(hashtb_t *me)
{
    struct ht_node *tpos;
    struct hlist_node *n;
    int i;
    int cnt = 0;

    if(!me->node_dump) {
        bwlist_debug("not register dump\n");
        return -1;
    }

    for(i = 0; i < me->slot_mask + 1; i++) {
        hlist_for_each_entry_safe(tpos, n, &me->head[i].hlist, hlist) {
            cnt++;
            me->node_dump(tpos->extend->data);
        }
    }
    return cnt;
}

int ht_cleanup_nodes(hashtb_t *me)
{
    struct ht_node *tpos;
    struct hlist_node *n;
    int i;
    int cnt = 0;

    rcu_read_lock_bh();
    for(i = 0; i < me->slot_mask + 1; i++) {
        hlist_for_each_entry_safe(tpos, n, &me->head[i].hlist, hlist) {
            cnt++;
            hlist_del_rcu(&tpos->hlist);

            call_rcu_bh(&tpos->rcu, ht_rcu_cb);
        }
    }
    rcu_read_unlock_bh();
    return cnt;
}

void ht_destroy(hashtb_t *me)
{
    struct ht_node *tpos;
    struct hlist_node *n;
    int i;

    del_timer(&me->timer);

    for(i = 0; i < me->slot_mask + 1; i++) {
        hlist_for_each_entry_safe(tpos, n, &me->head[i].hlist, hlist) {
            hlist_del_rcu(&tpos->hlist);
#if 0
            call_rcu_bh(&tpos->rcu, ht_rcu_cb);
#else
            printk("destroy one begin..\n");
            ht_node_destroy(tpos);
#endif
        }
    }
    synchronize_rcu_bh();

    if(atomic_read(&me->node_cnt) != 0 || atomic_read(&me->ext_cnt) != 0) {
        ht_debug("ht `%s` node cnt: %d, ext cnt: %d\n", me->name, atomic_read(&me->node_cnt), atomic_read(&me->ext_cnt));
        ht_debug("################# error ################\n");
    }

    ht_headpool_destroy(me);

    kmem_cache_destroy(me->data_hcache);
    kmem_cache_destroy(me->node_hcache);

    kfree(me);
}

hashtb_t *ht_create(char *name, uint32_t max_node_cnt, uint32_t node_size,
                    uint32_t check_interval, uint32_t check_cnt,
                    fn_node_key_match key_match,        /* 注意: 返回1表示匹配成功，0表示失败 */
                    fn_node_value_match value_match,    /* 注意: 返回1表示匹配成功，0表示失败 */
                    fn_node_hash_key hash_key,
                    fn_node_timeout_check timeout_check,
                    fn_node_cleanup cleanup,
                    fn_node_dump node_dump)
{
    struct hashtb *ht;
    uint32_t slot_num;
    char cache_name[12] = {0};

    ht = kzalloc(sizeof(*ht), GFP_KERNEL);
    if(!ht) {
        ht_error("ht alloc failed\n");
        return NULL;
    }

    slot_num = rounddown_pow_of_two(max_node_cnt);
    ht->slot_mask = slot_num - 1;
    if(ht_headpool_create(ht, slot_num)) {
        ht_error("ht alloc for headlist failed\n");
        goto err;
    }

    get_random_bytes(&ht->hash_salt, sizeof(ht->hash_salt));
    atomic_set(&ht->node_cnt, 0);
    atomic_set(&ht->ext_cnt, 0);

    ht->name = name;
    ht->node_data_size = node_size;

    snprintf(cache_name, sizeof(cache_name) - 1, "htnd_%s", name);
    ht->node_hcache = kmem_cache_create(cache_name, sizeof(struct ht_node), 0,
                                        SLAB_HWCACHE_ALIGN | SLAB_PANIC, NULL);
    if(!ht->node_hcache) {
        ht_error("ht create node memcache failed\n");
        goto err;
    }

    snprintf(cache_name, sizeof(cache_name) - 1, "htda_%s", name);
    ht->data_hcache = kmem_cache_create(cache_name, sizeof(ht_ext_t) + node_size, 0,
                                        SLAB_HWCACHE_ALIGN | SLAB_PANIC, NULL);
    if(!ht->data_hcache) {
        ht_error("ht create data memcache failed\n");
        goto err;
    }

    ht->check_interval = check_interval;
    ht->check_cnt = check_cnt;

    ht->node_cleanup = cleanup;
    ht->node_dump = node_dump;
    ht->node_key_match = key_match;
    ht->node_value_match = value_match;
    ht->node_hash_key = hash_key;
    ht->node_timeout_check = timeout_check;
    if(!key_match || !value_match || !hash_key || !timeout_check) {
        ht_error("please fix key_match & value_cmp & hash on hashtable %s\n", ht->name);
        goto err;
    }

    init_timer(&ht->timer);
    ht->timer.data = (unsigned long)ht;
    ht->timer.expires = jiffies + check_interval;
    ht->timer.function = ht_timer;
    add_timer(&ht->timer);

    return ht;
err:
    if(ht->data_hcache) {
        kmem_cache_destroy(ht->data_hcache);
    }
    if(ht->node_hcache) {
        kmem_cache_destroy(ht->node_hcache);
    }
    if(ht->head) {
        ht_headpool_destroy(ht);
    }
    if(ht) {
        kfree(ht);
    }
    return NULL;
}

int ht_node_cnt(hashtb_t *tb)
{
    return atomic_read(&tb->node_cnt);
}

int ht_ext_cnt(hashtb_t *tb)
{
    return atomic_read(&tb->ext_cnt);
}

