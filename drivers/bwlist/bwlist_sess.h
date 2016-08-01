#ifndef _BWLIST_SESS_H__
#define _BWLIST_SESS_H__


#define BWLIST_SESS_DETECTING       0
#define BWLIST_SESS_DROP            1
#define BWLIST_SESS_ACCEPT          2


typedef struct bwlist_sess bwlist_sess_t;


int bwlist_sess_init(uint32_t max_sess_cnt);
void bwlist_sess_fini(void);
void bwlist_sess_invalid_all(void);
void bwlist_sess_dump(void);
int bwlist_sess_node_cnt(void);


int bwlist_sess_set_result(uint32_t lan_ip, uint32_t wan_ip, uint16_t lan_port,
                           uint16_t wan_port, uint8_t protocol, uint8_t result);
int bwlist_sess_fresh(uint32_t lan_ip, uint32_t wan_ip,
                      uint16_t lan_port, uint16_t wan_port,
                      uint8_t protocol);


#endif //#ifndef _BWLIST_SESS_H__

