#include <linux/fs.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/inet.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>

#include "bwlist_public.h"
#include "bwlist_comm.h"
#include "bwlist_sess.h"
#include "bwlist_dnscache.h"

#define PROC_NAME       "bwlist"
#define PROC_DEBUG_NAME "debug_bwlist"

static int bwlist_open(struct inode *inode, struct file *file);
static int bwlist_release(struct inode *inode, struct file *file);
static long bwlist_ioctl(struct file *file, unsigned int cmd, unsigned long data);
static ssize_t bwlist_write(struct file *file, const char __user *buf, size_t count, loff_t *off);
static ssize_t bwlist_read(struct file *filp, char *buf, size_t count, loff_t *off);


static atomic_t g_reenter = ATOMIC_INIT(0);

static struct proc_dir_entry *proc_bwlist;
static struct file_operations bwlist_fops = {
    .unlocked_ioctl = bwlist_ioctl,
    .compat_ioctl = bwlist_ioctl,
    .open = bwlist_open,
    .release = bwlist_release,
};

static struct proc_dir_entry *proc_bwlist_debug;
static struct file_operations bwlist_fops_debug = {
    .read = bwlist_read,
    .write = bwlist_write,
};


int bwlist_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    if(atomic_inc_return(&g_reenter) > 1) {
        bwlist_debug("open value: %d\n", atomic_read(&g_reenter));
        atomic_dec(&g_reenter);
        ret = -EBUSY;
    }
    bwlist_debug("bwlist_open in, ret = %d\n", ret);
    return ret;
}


int bwlist_release(struct inode *inode, struct file *file)
{
    bwlist_debug("bwlist_release in\n");

    bwlist_turn_off();

    atomic_dec(&g_reenter);
    return 0;
}

ssize_t bwlist_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{
    char kbuf[512] = {0};
    char *ptr;
    int cnt;

    if(count >= sizeof(kbuf)) {
        bwlist_debug("write too many data, cut it off to len: %d\n", (int)sizeof(kbuf) - 1);
        cnt = sizeof(kbuf) - 1;
    } else {
        cnt = count;
    }

    if(copy_from_user(kbuf, buf, cnt)) {
        bwlist_debug("copy from user failed\n");
        return -EPERM;
    }

    ptr = strim(kbuf);

#define const_str_size(str) (sizeof(str)-1)
    if(!memcmp(ptr, "dump_sess", const_str_size("dump_sess"))) {
        bwlist_sess_dump();
    } else if(!memcmp(ptr, "dump_dip", const_str_size("dump_dip"))) {
        bwlist_dnsIP_dump();
    } else if(!memcmp(ptr, "open_debug", const_str_size("open_debug"))) {
        extern uint32_t g_verbose;
        g_verbose = 1;
    } else if(!memcmp(ptr, "close_debug", const_str_size("close_debug"))) {
        extern uint32_t g_verbose;
        g_verbose = 0;
    } else if(!memcmp(ptr, "show_list", const_str_size("show_list"))) {
        bwlist_dump(NULL, -1);
    } else if(!memcmp(ptr, "show_exclusive", const_str_size("show_exclusive"))) {
        bwlist_exclusive_dump(NULL, -1);
    }
    return count;
}

ssize_t bwlist_read(struct file *filp, char *buf, size_t count, loff_t *off)
{
    extern uint32_t g_verbose;
    int len = 0;
    if (*off != 0) {
        return 0;
    }

    len += snprintf(buf + len, count - len, "log debug %s\n", g_verbose ? "on" : "off");
    len += snprintf(buf + len, count - len, "sess node cnt: %d\n", bwlist_sess_node_cnt());
    len += snprintf(buf + len, count - len, "DNS node cnt: %d\n", bwlist_dnsIP_node_cnt());
    len += snprintf(buf + len, count - len, "DNS expired reserved: %ld(s)\n",
                    bwlist_dnsIP_get_expire_time());

    len += snprintf(buf + len, count - len, "cmd: \n");
    len += snprintf(buf + len, count - len, "\tdump_sess\n");
    len += snprintf(buf + len, count - len, "\tdump_dip\n");
    len += snprintf(buf + len, count - len, "\topen_debug\n");
    len += snprintf(buf + len, count - len, "\tclose_debug\n");
    len += snprintf(buf + len, count - len, "\tshow_list\n");
    len += snprintf(buf + len, count - len, "\tshow_exclusive\n");


    len += bwlist_dump(buf + len, count - len);
    len += bwlist_exclusive_dump(buf + len, count - len);

    *off += len;
    return len;
}

long bwlist_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
    switch(cmd) {
        case BWLIST_IOCTL_TURN_ON:
            bwlist_turn_on(data);
            break;

        case BWLIST_IOCTL_TURN_OFF:
            bwlist_turn_off();
            break;

        case BWLIST_IOCTL_ADD_GROUP:
            return bwlist_add_grp(data);

        case BWLIST_IOCTL_CLEAN_GROUP:
            bwlist_clean_grp();
            break;

        case BWLIST_IOCTL_GET_STATUS:
            break;

        case BWLIST_IOCTL_UPD_WATCH_DEV:
            return bwlist_upd_watch_dev(data);

        case BWLIST_IOCTL_UPD_EXCLUSIVE_PROT:
            return bwlist_upd_exclusive_proto(data);

        case BWLIST_IOCTL_UPD_DEFAULT_URL:
            return bwlist_upd_default_url(data);

        case BWLIST_IOCTL_UPD_DNS_EXPIRED:
            bwlist_dnsIP_set_expire_time(data);
            break;

        default:
			bwlist_debug("bad cmd: %u\n", cmd);
            return -EPERM;
    }
    return 0;
}


int bwlist_interface_init(void)
{
    proc_bwlist = proc_create(PROC_NAME, S_IRUSR | S_IWUSR, NULL, &bwlist_fops);
    if(!proc_bwlist) {
        bwlist_debug("create /proc/%s entry failed\n", PROC_NAME);
        return -1;
    }

    proc_bwlist_debug = proc_create(PROC_DEBUG_NAME, S_IRUSR | S_IWUSR, NULL, &bwlist_fops_debug);
    if(!proc_bwlist_debug) {
        bwlist_debug("create /proc/%s entry failed\n", PROC_DEBUG_NAME);

        remove_proc_entry(PROC_NAME, NULL);
        return -1;
    }

    return 0;
}

void bwlist_interface_fini(void)
{
    remove_proc_entry(PROC_NAME, NULL);
    remove_proc_entry(PROC_DEBUG_NAME, NULL);

    bwlist_debug("bwlist_interface_fini over...\n");
}

