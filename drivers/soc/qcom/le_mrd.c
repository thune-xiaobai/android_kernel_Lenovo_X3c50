/*
    mini ramdump module

    This is lenovo midh mrd driver implement.

    Author:Kerry Xi
    Date: Sep, 2013
    Copy Right: Lenovo 2015

    history:
    Jul 2015, Kerry Xi,  Add subsystem silent crash report
    Jun 2015, Kerry Xi,  Support mrd in aarch64
    Sep 2013, Kerry Xi,  Initial the mrd driver
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/proc_fs.h>
#include "le_mrd.h"
#include <soc/qcom/smem.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/notifier.h>
#include <soc/qcom/subsystem_notif.h>

//imem flag for bootloader that store the MRD_SMEM_TRIGGER_MAGIC_NUMBER flag
#define FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET 0xC   //reuse the uefi flag

//the mrd shareindex table physical address
#define FIX_IMEM_MRD_DUMP_TABLE_ADDR 0x34             //use a new imem flag section

//share index pair variable that defines both the pa and va of shareindex memory
static le_mrd_shareindex_pair mrd_shareindex_data;

extern void imem_write(int offset,unsigned int val);

void mrd_set_mrdmode(int mode);
static int mrd_init_status=0;

/*#define MRD_USE_SMEM*/
static void * mrd_alloc_share_index(int size)
{
	void *ret = NULL;
#ifdef MRD_USE_SMEM
	ret = smem_find(SMEM_ID_VENDOR_MRD_INDEX,size,0,SMEM_ANY_HOST_FLAG);
	pr_info("%s:smem_find ret=%p",__func__,ret);
	ret = smem_alloc(SMEM_ID_VENDOR_MRD_INDEX,size,0,SMEM_ANY_HOST_FLAG);
	if (!ret) {
		pr_info("%s:smem_alloc fail, try smem_alloc2\n",__func__);
		return NULL;
	}
	pr_info("%s:pa=%llx\n",__func__,smem_virt_to_phys(ret));
#else
	ret = kzalloc(size, GFP_KERNEL);
#endif
	return ret;
}

static inline unsigned long get_physical_address(unsigned long va)
{
#ifdef MRD_USE_SMEM
	return smem_virt_to_phys((void*)va);
#else
	return virt_to_phys((void*)va);
#endif
}

int boot_rtc_seconds=0;

static int mrd_debug=0;

static int __init mrd_setup_debug(char *str)
{
	unsigned int val = memparse(str, &str);

	if (val)
		mrd_debug = val;

	return 0;
}

early_param("mrd_dbg",mrd_setup_debug);

#ifdef MRD_USE_SMEM
void *zmemset(void *s,int c,size_t count)
{
	char *xs = s;
	while (count --)
		*xs++ = c;
	return s;
}
#else
#define zmemset memset
#endif

//register one shareindex before the mrd driver initialized
#define MAX_MRD_EARLY_SHAREINDEX 3
static int early_si_count=0;
static mrd_shareindex_item_t mrd_index_early[MAX_MRD_EARLY_SHAREINDEX] ;

static int mrd_register_shareindex_early(mrd_shareindex_item_t* index)
{
	if (early_si_count >= MAX_MRD_EARLY_SHAREINDEX)
	{
		pr_info("%s: mrd early index full, current count is %d\n",__func__,early_si_count);
		return -1;
	}
	memcpy(&mrd_index_early[early_si_count],index,sizeof(*index));
	early_si_count++;
	return 0;
}

static int mrd_flush_shareindex_early(void)
{
	int i;

	for (i=0; i<early_si_count; i++)
	{
		mrd_register_shareindex(&mrd_index_early[i]);
	}
	return 0;
}

//register one shareindex item for external module
int mrd_register_shareindex_name(char* name, void* va, int size, unsigned int param1)
{
	mrd_shareindex_item_t  index;

	//register the dmesg log info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, name, MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(va);
	index.size = size;
	index.param1 = param1;

	if (!mrd_init_status) {
		pr_info("mrd is not initialized,move %s into early area\n",name);
		mrd_register_shareindex_early(&index);
		return -1;
	}

	return mrd_register_shareindex(&index);
}

//register one shareindex item for external module by physical address,such as dma memory
int mrd_register_shareindex_name_phys(char* name, unsigned int pa, int size, unsigned int param1)
{
	mrd_shareindex_item_t  index;

	//register the dmesg log info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, name, MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = pa;
	index.size = size;
	index.param1 = param1;

	if (!mrd_init_status) {
		pr_info("mrd is not initialized,move %s into early area\n",name);
		mrd_register_shareindex_early(&index);
		return -1;
	}

	return mrd_register_shareindex(&index);
}

//register one shareindex
int mrd_register_shareindex(mrd_shareindex_item_t* index)
{
	mrd_shareindex_item_t* avail_index = NULL;
	if ( (mrd_shareindex_data.shareindex_table_ptr)->index_count == (mrd_shareindex_data.shareindex_table_ptr)->total_count) {
		pr_err("%s: the shareindex area is full, can not register new index\n", __func__);
		return -1;
	}

	if (index->name[MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH-1]) {
		pr_err("%s:the mrd shareindex name is too long.\n",__func__);
		return -1;
	}

	avail_index = (mrd_shareindex_item_t* ) ((unsigned char*)mrd_shareindex_data.shareindex_table_ptr
			+ mrd_shareindex_data.shareindex_table_ptr->head_size
			+ sizeof(mrd_shareindex_item_t)*mrd_shareindex_data.shareindex_table_ptr->index_count);

	mrd_shareindex_data.shareindex_table_ptr->index_count++;

	*avail_index = *index;

	avail_index->subsys = MRD_SUBSYS_KRAIT;
	if (mrd_debug)
		pr_info("mrd register si item %s, size %d\n",index->name, index->size);
	if (mrd_debug)
		pr_info("%s:register shareindex item: \n"
				"  index va  :0x%p\n"
				"      name  :%s\n"
				"      pa    :0x%08x\n"
				"      size  :0x%x\n"
				"      subsys:%d\n",
				__func__,
				avail_index,
				index->name,
				index->address,
				index->size,
				avail_index->subsys);
	return 0;
};

//default value
static int mrd_mode=MRD_MODE_DLOADMODE;

//when normal reboot or reboot to dloadmode, it will enter into normal dloadmode.
void mrd_unset_miniramdump(void)
{
	mrd_set_mrdmode(MRD_MODE_IGNORE);
	imem_write(FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET,0);
	imem_write(FIX_IMEM_MRD_DUMP_TABLE_ADDR, 0);
	pr_err( "%s: unset the mrdmode\n",__func__);
}

/*set system mrd mode
param:
1: enter into mrd mode, and reboot after collect mrd info
0: enter into mrd mode, and enter into dload mode after collect mrd info
*/
void mrd_set_mrdmode(int mode)
{
	mrd_shareindex_header_t *table;
	table = mrd_shareindex_data.shareindex_table_ptr;
	if  ( (mode == MRD_MODE_REBOOTMODE)
		|| (mode == MRD_MODE_DLOADMODE)
		|| (mode == MRD_MODE_IGNORE))
		table->mrdmode = mode;
	else
		pr_err("%s:Error:invalid mode parameter %d\n",__func__,mode);

	pr_err("%s:set the mrdmode %s,value=%d\n",__func__,mode?"true":"false",table->mrdmode);
}

//the module parameter set interface for mrd_mode
//param: 1: download mode
//       2: mrd mode
static int mrd_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = mrd_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not one or two, ignore. */
	if ( (mrd_mode < 0) || (mrd_mode >2)) {
		pr_err("input invlaid mrd mode %d, restore old value %d\n",mrd_mode,old_val);
		mrd_mode = old_val;
		return -EINVAL;
	}

	mrd_set_mrdmode(mrd_mode);

	return 0;
}
module_param_call(mrd_download_mode, mrd_mode_set, param_get_int,
			&mrd_mode, 0644);

//the module parameter set interface for mrd_mode
//param: 1: debug mode
//       0: non debug mode
static int mrd_debug_info_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = mrd_debug;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	pr_info("new mrd_debug is:%d,old value is:%d\n",mrd_debug,old_val);

	return 0;
}
module_param_call(mrd_debug_mode, mrd_debug_info_set, param_get_int,
			&mrd_debug, 0644);

//print the shareindex info
static void mrd_dump_shareindex_info(void)
{
	mrd_shareindex_header_t *table;
	mrd_shareindex_item_t* cur_index = NULL;
	int i=0;

	table = mrd_shareindex_data.shareindex_table_ptr;

	//dump the header
	pr_info("%s: dump the shareindex table info:\n", __func__);
	pr_info("  shareindex table va=%p, pa=0x%08x\n",mrd_shareindex_data.shareindex_table_ptr, mrd_shareindex_data.shareindex_table_phys);
	pr_info("        signature  :0x%08x\n",table->signature);
	pr_info("        version    :0x%x\n",table->version);
	pr_info("        total_size :0x%08x\n",table->total_size);
	pr_info("        head_size  :0x%08x\n",table->head_size);
	pr_info("        total_count:0x%08x\n",table->total_count);
	pr_info("        index_count:0x%08x\n",table->index_count);
	pr_info("        lock       :0x%08x\n",table->lock);
	pr_info("        mrdmode    :0x%08x\n",table->mrdmode);
	pr_info("        boot_rtc   :0x%08x\n",table->boot_rtc);

	//dump the items
	for (i=0; i<table->index_count; i++) {
		cur_index = (mrd_shareindex_item_t* ) ((unsigned char *)mrd_shareindex_data.shareindex_table_ptr
				+ mrd_shareindex_data.shareindex_table_ptr->head_size
				+ sizeof(mrd_shareindex_item_t)*i);
		pr_info("  No. [%d] shareindex item:\n",i);
		pr_info("        indexva :0x%p\n",cur_index);
		pr_info("        name    :%s\n",cur_index->name);
		pr_info("        address :0x%08x\n",cur_index->address);
		pr_info("        subsyss :0x%08x\n",cur_index->subsys);
		pr_info("        size    :%d\n",cur_index->size);
		pr_info("        param1  :%d\n",cur_index->param1);
		pr_info("        param2  :%d\n",cur_index->param2);
	}
}

#define MRD_DEBUG_FORCE_TRIGGER "force_trigger"
#define MRD_DEBUG_FORCE_SSR_SET "force_ssr_set"
/* Define kernel_log to add a sysfs interface to read/write */
static ssize_t debugfs_mrd_debug_read(struct file *file,char __user *buf, size_t n, loff_t *ppos)
{
	if (*ppos == 0)
		printk("mrd:---kernel_log read/wrte testing--- pos=%d\n",(int)*ppos);

	mrd_dump_shareindex_info();
	return 0;
}

static void mrd_report_subsys_uevent(char *subsys_name,char *reason);
static ssize_t debugfs_mrd_debug_write(struct file *file,const char* buffer,
		size_t count, loff_t *ppos)
{
	char buf[256];
	int ret;
	size_t len = strnlen(MRD_DEBUG_FORCE_TRIGGER , sizeof(MRD_DEBUG_FORCE_TRIGGER ));

	if (count < sizeof(MRD_DEBUG_FORCE_TRIGGER )) {
		pr_info("the command length is not valid\n");
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, len)) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (!strncmp(buf, MRD_DEBUG_FORCE_TRIGGER, len)) {
		//set the sbl force trigger debug magic
		imem_write(FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET,MRD_FORCE_TRIGGER_MAGIC);
		printk("write force_trigger to mrd debug magic flag\n");
		return count;
	} else if (!strncmp(buf,MRD_DEBUG_FORCE_SSR_SET,len)) {
		mrd_report_subsys_uevent("modem","simulate subsystem silent crash uevent");
		return count;
	} else {
		pr_info("the current command %s is not valid.\n",buf);
		ret = -EINVAL;
		goto write_proc_failed;
	}

write_proc_failed:
	printk("echo force_trigger > mrd_debug\n");

	return ret;
}

static const struct file_operations mrd_debug_kernel_log_fops = {
	.owner = THIS_MODULE,
	.read = debugfs_mrd_debug_read,
	.write = debugfs_mrd_debug_write,
};

#define MAX_MRD_SW_VERSION_LEN 128
static char mrd_sw_ver[MAX_MRD_SW_VERSION_LEN];
static ssize_t debugfs_mrd_sw_ver_write(struct file *file, const char __user *buf,
					size_t count, loff_t *ppos)
{
	int len=0;
	if (IS_ERR(file) || file == NULL) {
		pr_err("mrd Function Input Error %ld\n", PTR_ERR(file));
		return -ENOMEM;
	}

	len = strlen(mrd_sw_ver);
	if ( len ) {
		pr_err("mrd sw version has been update\n");
		return -EFAULT;
	}

	if (count < MAX_MRD_SW_VERSION_LEN) {
		if (copy_from_user(mrd_sw_ver, (void __user *) buf, count))
			return -EFAULT;
		pr_info("mrd update the sw_ver %s\n",mrd_sw_ver);
	} else {
		pr_err("mrd Error input software version");
		return -ENOMEM;
	}

	return count;
}

static ssize_t debugfs_mrd_sw_ver_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	int output, rc;
	if (IS_ERR(file) || file == NULL) {
		pr_err("Function Input Error %ld\n", PTR_ERR(file));
		return -ENOMEM;
	}

	output = strlen(mrd_sw_ver);
	rc = simple_read_from_buffer((void __user *) buf, output, ppos,
					(void *) mrd_sw_ver, output);

	return rc;
}

static const struct file_operations mrd_sw_ver_fops = {
	.owner = THIS_MODULE,
	.read = debugfs_mrd_sw_ver_read,
	.write = debugfs_mrd_sw_ver_write,
};

static int mrd_init_debugfs(void)
{
	struct dentry *dir, *file, *file2;

	dir = debugfs_create_dir("le_mrd", NULL);
	if (!dir)
		return -ENOMEM;

	file = debugfs_create_file("mrd_debug", 0640, dir, NULL,
					&mrd_debug_kernel_log_fops);
	if (!file) {
		pr_err("%s:init mrd_debug fail\n",__func__);
		debugfs_remove(dir);
		return -ENOMEM;
	}
	file2 = debugfs_create_file("sw_ver", 0664, dir, NULL,
					&mrd_sw_ver_fops);
	if (!file2) {
		pr_err("%s:init sw_ver fail\n",__func__);
	}
	return 0;
}

static struct device *g_mrd_dev;
#define MAX_SSR_REASON_LEN 810
#define MAX_UEVNET_SSR_REASON (MAX_SSR_REASON_LEN + 32)

static char uevent_subsys_name[128];
static char uevent_ssr_reason[MAX_UEVNET_SSR_REASON];
static void mrd_report_subsys_uevent(char *subsys_name,char *reason)
{
	char *envp[4];
	envp[0] = "SSR_ID=mrd_ue";
	envp[1] = uevent_subsys_name;
	envp[2] = uevent_ssr_reason;
	envp[3] = NULL;
	memset(uevent_subsys_name,0,sizeof(uevent_subsys_name));
	memset(uevent_ssr_reason,0,sizeof(uevent_ssr_reason));

	snprintf(uevent_subsys_name,sizeof(uevent_subsys_name),"SSR_SUBSYS_NAME=%s",subsys_name);
	snprintf(uevent_ssr_reason,sizeof(uevent_ssr_reason),"SSR_REASON=%s",reason);
	pr_info("mrd report %s silent crash:%s\n",subsys_name,reason);
	kobject_uevent_env(&g_mrd_dev->kobj,KOBJ_CHANGE,envp);
}

static const char *notif_to_string(enum subsys_notif_type notif_type)
{
	switch (notif_type) {
	case	SUBSYS_BEFORE_SHUTDOWN:
		return __stringify(SUBSYS_BEFORE_SHUTDOWN);
	case	SUBSYS_AFTER_SHUTDOWN:
		return __stringify(SUBSYS_AFTER_SHUTDOWN);
	case	SUBSYS_BEFORE_POWERUP:
		return __stringify(SUBSYS_BEFORE_POWERUP);
	case	SUBSYS_AFTER_POWERUP:
		return __stringify(SUBSYS_AFTER_POWERUP);
	default:
		return "unknown";
	}
}

static char mrd_ssr_modem_reason[MAX_SSR_REASON_LEN];
void mrd_update_modem_restart_reason(char *reason)
{
	strlcpy(mrd_ssr_modem_reason,reason,MAX_SSR_REASON_LEN);
}

static int mrd_subsys_modem_notifier_call(struct notifier_block *this,
				  unsigned long code,
				  void *data)
{
	if (code == SUBSYS_AFTER_POWERUP) {
		printk(KERN_WARNING "mrd: Notification %s from modem subsystem\n",
				notif_to_string(code));
		if (strlen(mrd_ssr_modem_reason)) {
			pr_debug("there is modem silent crash:%s\n",mrd_ssr_modem_reason);
			mrd_report_subsys_uevent("modem",mrd_ssr_modem_reason);
			memset(mrd_ssr_modem_reason,0,sizeof(mrd_ssr_modem_reason));
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block nb_modem = {
	.notifier_call = mrd_subsys_modem_notifier_call,
};

static char mrd_ssr_adsp_reason[MAX_SSR_REASON_LEN];
void mrd_update_adsp_restart_reason(char *reason)
{
	strlcpy(mrd_ssr_adsp_reason,reason,MAX_SSR_REASON_LEN);
}

static int mrd_subsys_adsp_notifier_call(struct notifier_block *this,
				  unsigned long code,
				  void *data)
{
	if (code == SUBSYS_AFTER_POWERUP) {
		printk(KERN_WARNING "mrd: Notification %s from adsp subsystem\n",
				notif_to_string(code));
		if (strlen(mrd_ssr_adsp_reason)) {
			pr_debug("there is adsp silent crash:%s\n",mrd_ssr_adsp_reason);
			mrd_report_subsys_uevent("adsp",mrd_ssr_adsp_reason);
			memset(mrd_ssr_adsp_reason,0,sizeof(mrd_ssr_adsp_reason));
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block nb_adsp = {
	.notifier_call = mrd_subsys_adsp_notifier_call,
};

static void mrd_subsys_notif_reg_notifier(void)
{
	void *handle;

	memset(mrd_ssr_modem_reason,0,sizeof(mrd_ssr_modem_reason));
	handle = subsys_notif_register_notifier("modem", &nb_modem);
	pr_debug("%s: Registered modem notifier\n", __func__);

	memset(mrd_ssr_adsp_reason,0,sizeof(mrd_ssr_adsp_reason));
	handle = subsys_notif_register_notifier("adsp", &nb_adsp);
	pr_debug("%s: Registered adsp notifier\n", __func__);
}

static int mrd_probe(struct platform_device *pdev)
{
	if (!pdev->dev.of_node )
		return -ENODEV;

	pr_info("%s:create one mrd device\n",__func__);
	if (!g_mrd_dev)
		g_mrd_dev = &pdev->dev;

	mrd_subsys_notif_reg_notifier();
	return 0;
}


static int mrd_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id mrd_match_table[] = {
	{	.compatible = "lenovo,mrd-subsys",	},
	{}
};

static struct platform_driver mrd_driver = {
	.probe = mrd_probe,
	.remove = mrd_remove,
	.driver = {
		.name = "msm_mrd",
		.owner = THIS_MODULE,
		.of_match_table = mrd_match_table,
	},
};

void mrd_register_rtc_xtime(unsigned int);
void mrd_register_ptreg_sp(unsigned int tag);
static int __init init_mrd_shareindex(void)
{
	mrd_shareindex_header_t *table;
	mrd_shareindex_item_t  index;
	extern char __log_buf[];

	//alloc the mrd shareindex space from the smem
	mrd_shareindex_data.shareindex_table_ptr = mrd_alloc_share_index(MRD_SHAREINDEX_SIZE);
	if (!mrd_shareindex_data.shareindex_table_ptr)
	{
		pr_info("%s:alloc shareindex memory fail,size=%d!\n",__func__,MRD_SHAREINDEX_SIZE);
		return -1;
	}
	pr_info("%s:init the mrd shareindex table ptr=%p\n",__func__,mrd_shareindex_data.shareindex_table_ptr);
	/*if (mrd_debug)*/
	/*return 0;*/
	table = mrd_shareindex_data.shareindex_table_ptr;
	memset(table,0,MRD_SHAREINDEX_SIZE);

	//init the shareindex header info
	table->signature = MRD_SHAREINDEX_HEAD_MAGIC;
	table->version = MRD_SHAREINDEX_VERSION;
	table->total_size = MRD_SHAREINDEX_SIZE;
	table->head_size = MRD_SHAREINDEX_HEAD_SIZE;
	table->total_count = (MRD_SHAREINDEX_SIZE - MRD_SHAREINDEX_HEAD_SIZE) / sizeof(mrd_shareindex_item_t);
	table->index_count = 0;
	table->lock = 0;
	table->mrdmode = mrd_mode;
	table->boot_rtc = boot_rtc_seconds;    //need to update

	//get the shareindex table physical from smem va to pa
	mrd_shareindex_data.shareindex_table_phys = get_physical_address((unsigned long)mrd_shareindex_data.shareindex_table_ptr);
	pr_info("%s:mrd dump table address=0x%08x\n",__func__,(u32)mrd_shareindex_data.shareindex_table_phys);

	//setup the imem flag
	imem_write(FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET,MRD_SMEM_TRIGGER_MAGIC);
	imem_write(FIX_IMEM_MRD_DUMP_TABLE_ADDR, mrd_shareindex_data.shareindex_table_phys);
	imem_write(FIX_IMEM_MRD_DUMP_DONE_FLAG_ADDR , 0);
	pr_info( "%s: mrd shareindex table set up\n",__func__);
	if (mrd_debug)
		pr_info( "%s: mrd shareindex table set up\n",__func__);

	//register the linux_banner info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "banner.txt", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(linux_banner);
	index.size = strlen(linux_banner);
	index.param1 = MRD_SHAREINDEX_SPECIAL_TAG;
	mrd_register_shareindex(&index);

	//register the command_line info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "cmdline.txt", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(saved_command_line);
	index.size = strlen(saved_command_line);
	index.param1 = MRD_SHAREINDEX_SPECIAL_TAG;
	mrd_register_shareindex(&index);

	//register the sw_ver info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "sw_ver.txt", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(mrd_sw_ver);
	index.size = MAX_MRD_SW_VERSION_LEN;
	index.param1 = MRD_SHAREINDEX_SPECIAL_TAG;
	mrd_register_shareindex(&index);

	//register the dmesg log info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "LASTKMSG.txt", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(__log_buf);
	index.size = 1 << (CONFIG_LOG_BUF_SHIFT);
	mrd_register_shareindex(&index);

	//register the qsee info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "qseebk.bin", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = 0x6DB2000;      //from backup area part to end
	index.size = 0x14E000;
	mrd_register_shareindex(&index);

	//register the qhee info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "qheebk.bin", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = 0x6C20000;      //just the backuped qhee memory part
	index.size = 0x20000;
	mrd_register_shareindex(&index);

	mrd_init_debugfs();

	if (mrd_debug)
		mrd_dump_shareindex_info();
	mrd_init_status = 1;
	mrd_register_rtc_xtime(MRD_SHAREINDEX_SPECIAL_TAG);
	mrd_register_ptreg_sp(MRD_SHAREINDEX_VA_TAG);

	//register all early pre-registered shareindex
	mrd_flush_shareindex_early();
	return platform_driver_register(&mrd_driver);
}

static void __exit mrd_exit(void)
{
	platform_driver_unregister(&mrd_driver);
}

//module_init(init_mrd_shareindex);
device_initcall_sync(init_mrd_shareindex);
//late_initcall(init_mrd_shareindex);
module_exit(mrd_exit)
