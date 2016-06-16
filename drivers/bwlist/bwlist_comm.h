#ifndef _BWLIST_COMM_H__
#define _BWLIST_COMM_H__

#include <linux/if.h>
#include <linux/jiffies.h>
#include "bwlist_public.h"

void bwlist_turn_on(unsigned long arg);
void bwlist_turn_off(void);
int bwlist_add_grp(unsigned long data);
void bwlist_clean_grp(void);
int bwlist_upd_watch_dev(unsigned long data);
int bwlist_upd_exclusive_proto(unsigned long data);
int bwlist_upd_default_url(unsigned long arg);



int is_bwlist_on(void);
uint32_t bwlist_grp_magic(void);


bwlist_group_t *bwlist_find_grp_with_key(const unsigned char *domain);

int bwlist_dump(char *buf, int size);
int bwlist_exclusive_dump(char *buf, int size);


#define MAX_KMALLOC_SIZE (64*1024)

#define NIPQUAD(nip) \
    ((unsigned char *)&nip)[0], \
    ((unsigned char *)&nip)[1], \
    ((unsigned char *)&nip)[2], \
    ((unsigned char *)&nip)[3]

#define HIPQUAD(hip) \
    ((unsigned char *)&hip)[3], \
    ((unsigned char *)&hip)[2], \
    ((unsigned char *)&hip)[1], \
    ((unsigned char *)&hip)[0]


void hex_printout(const char *msg, const unsigned char *buf, unsigned int len);

#define bwlist_debug(fmt, args...) \
    do { \
        extern uint32_t g_verbose; \
        if(g_verbose) { \
            printk(KERN_EMERG "# bwlist # %s(%d)(%lu): " fmt, __FUNCTION__, __LINE__, jiffies, ##args); \
        } \
    } while(0)

#endif //#ifndef _BWLIST_COMM_H__

