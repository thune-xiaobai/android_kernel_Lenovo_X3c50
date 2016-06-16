#ifndef _BWLIST_PUBLIC_H__
#define _BWLIST_PUBLIC_H__

#include <asm/ioctl.h>
//#include <linux/if.h>

#define BWLIST_MAGIC            0x20150527

#define MAX_DEV_NAME_LEN        16    /* => IFNAMSIZ */
#define MAX_EXCLUSIVE_DEV_CNT   10
#define MAX_REDIRECT_URL_LEN    256
#define MAX_PORT_NUM            6
#define MAX_KEYWORD_SIZE        64
#define MAX_IP_CNT              (MAX_KEYWORD_SIZE/sizeof(struct bwlist_ip))

typedef struct bwlist_ip {
    uint32_t                ip_base;
    uint32_t                ip_mask;
} bwlist_ip_t;

typedef struct bwlist_port {
    uint8_t                 udp_port_num;
    uint8_t                 tcp_port_num;
    uint8_t                 tcp_port_flag;
    uint8_t                 udp_port_flag;

    uint16_t                udp_begin_port;
    uint16_t                udp_end_port;

    uint16_t                tcp_begin_port;
    uint16_t                tcp_end_port;

    uint16_t                udp_port[MAX_PORT_NUM];
    uint16_t                tcp_port[MAX_PORT_NUM];

} bwlist_port_t;

/* 保证64字节对齐 */
#define BWLIST_ITEM_TYPE_IPPORT     0x1
#define BWLIST_ITEM_TYPE_KEYPORT    0x2
typedef struct bwlist_item {
    uint32_t                type;
    bwlist_port_t           port;
    union {
        bwlist_ip_t         ip[MAX_IP_CNT];
        char                key[MAX_KEYWORD_SIZE];
    };
} bwlist_item_t;


/* 保证64字节对齐 */
typedef struct bwlist_group {
    uint32_t                    magic;
    uint32_t                    item_cnt;
    char                        redirect_url[MAX_REDIRECT_URL_LEN];
    bwlist_item_t               item[0];
} bwlist_group_t;


#define BWLIST_TYPE_BLACK_LIST 0x0
#define BWLIST_TYPE_WHITE_LIST 0x1


typedef struct device_name {
    char                    name[MAX_DEV_NAME_LEN];
} device_name_t;
typedef struct bwlist_watch_dev {
    uint32_t                magic;
    uint32_t                count;
    device_name_t           dev[0];
} bwlist_watch_dev_t;



typedef struct bwlist_exclusive_proto {
    uint32_t                magic;
    uint32_t                count;
    uint8_t                 proto[0];
} bwlist_exclusive_proto_t;



#define BWLIST_IOCTL_TYPE               'w'
#define BWLIST_IOCTL_TURN_ON            _IOWR(BWLIST_IOCTL_TYPE, 1, uint32_t)
#define BWLIST_IOCTL_TURN_OFF           _IOWR(BWLIST_IOCTL_TYPE, 2, uint32_t)
#define BWLIST_IOCTL_ADD_GROUP          _IOWR(BWLIST_IOCTL_TYPE, 3, uint32_t)
#define BWLIST_IOCTL_CLEAN_GROUP        _IOWR(BWLIST_IOCTL_TYPE, 4, uint32_t)
#define BWLIST_IOCTL_GET_STATUS         _IOWR(BWLIST_IOCTL_TYPE, 5, uint32_t)
#define BWLIST_IOCTL_UPD_WATCH_DEV      _IOWR(BWLIST_IOCTL_TYPE, 6, uint32_t)
#define BWLIST_IOCTL_UPD_EXCLUSIVE_PROT _IOWR(BWLIST_IOCTL_TYPE, 7, uint32_t)
#define BWLIST_IOCTL_UPD_DEFAULT_URL    _IOWR(BWLIST_IOCTL_TYPE, 8, uint32_t)
#define BWLIST_IOCTL_UPD_DNS_EXPIRED    _IOWR(BWLIST_IOCTL_TYPE, 9, uint32_t)


#endif //#ifndef _BWLIST_PUBLIC_H__

