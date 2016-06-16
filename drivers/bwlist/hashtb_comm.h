#ifndef _HASHTB_COMM_H__
#define _HASHTB_COMM_H__

#include <linux/types.h>

typedef struct hashtb hashtb_t;



typedef uint32_t (*fn_node_hash_key)(void *node_data, uint32_t size);
/* 注意: 返回1表示匹配成功，0表示失败 */
typedef int  (*fn_node_key_match)(void *node_data1, void *node_data2);
/* 注意: 返回1表示匹配成功，0表示失败 */
typedef int  (*fn_node_value_match)(void *node_data1, void *node_data2);
/* 注意: 返回1表示超时，0表示未超时 */
typedef int (*fn_node_timeout_check)(void *node_data);

typedef void (*fn_node_cleanup)(void *node_data);
typedef void (*fn_node_dump)(void *node_data);


void *ht_data_create(hashtb_t *tb);
void ht_data_destroy(hashtb_t *tb, void *data, uint32_t size);

int ht_cleanup_nodes(hashtb_t *tb);
int ht_dump_all(hashtb_t *tb);

int ht_node_update(hashtb_t *tb, void *data, uint32_t size);
void *ht_node_find(struct hashtb *tb, void *data, uint32_t size);

int ht_node_cnt(hashtb_t *tb);
int ht_ext_cnt(hashtb_t *tb);


void ht_destroy(hashtb_t *me);
hashtb_t *ht_create(char *name, uint32_t max_node_cnt, uint32_t node_size,
                    uint32_t check_interval, uint32_t check_cnt,
                    fn_node_key_match   key_match, /* 注意: 返回1表示匹配成功，0表示失败 */
                    fn_node_value_match value_match, /* 注意: 返回1表示匹配成功，0表示失败 */
                    fn_node_hash_key    hash_key,
                    fn_node_timeout_check timeout_check,
                    fn_node_cleanup     cleanup,
                    fn_node_dump        node_dump
                   );

#endif //#ifndef _HASHTB_COMM_H__

