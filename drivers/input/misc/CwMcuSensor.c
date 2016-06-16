#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h> /* BUS_I2C */
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "CwMcuSensor.h"
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/irq_work.h>

#define SENSOR_HUB_TAG                  "[SensorHub] "
#define DEBUG                           1
#if defined(DEBUG)
#define SH_FUN(f)                       printk(KERN_INFO SENSOR_HUB_TAG"%s\n", __FUNCTION__)
#define SH_ERR(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_LOG(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_DBG(fmt, args...)            printk(KERN_INFO SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define SH_FUN(f)                       printk(KERN_INFO SENSOR_HUB_TAG"%s\n", __FUNCTION__)
#define SH_ERR(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_LOG(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_DBG(fmt, args...)
#endif


/* GPIO for MCU control */
#define GPIO_CW_MCU_WAKE_UP     916    //zhujp2 modify 67  38+878
#define GPIO_CW_MCU_BOOT        950     // 72+87
#define GPIO_CW_MCU_RESET       903   //zhujp2 modify 25+878
#define GPIO_CW_MCU_INTERRUPT   951   //zhujp2 modify 878+73
#define QueueSystemInfoMsgSize  30
#define QueueWarningMsgSize          30

#define ACK                     0x79
#define NACK                    0x1F

#define DPS_MAX         (1 << (16 - 1))

/* Input poll interval in milliseconds */


#define CWMCU_POLL_MAX      2000
#define FT_VTG_MIN_UV       1800000
#define FT_VTG_MAX_UV       1800000
#define FT_VTG_MIN_B_UV     285000  //zhujp2 add 
#define FT_VTG_MAX_B_UV     2850000  //zhujp2 add
/*
#define FT_VTG_MIN_C_UV     2950000  //zhujp2 add 
#define FT_VTG_MAX_C_UV     2950000  //zhujp2 add
*/
#define FT_I2C_VTG_MIN_UV   1800000
#define FT_I2C_VTG_MAX_UV   1800000

#define CWMCU_MAX_OUTPUT_ID     (CW_SNAP+1)
#define CWMCU_MAX_OUTPUT_BYTE       (CWMCU_MAX_OUTPUT_ID * 7)
#define CWMCU_MAX_DRIVER_OUTPUT_BYTE        256

/* turn on gpio interrupt if defined */
#define CWMCU_INTERRUPT
#define CWMCU_MUTEX

uint8_t Buf0[]={0x02,0x01,0x00,0x00,0x94,0x70,0x00,0x48,0x01,0x5A,0x00,0xAA,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x17,0x00,0x12,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x17,0x00,0x12,0x00,0x16,0x00,0x3E,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x17,0x00,0x12,0x00,0x16,0x00,0x3E,0x00,0x17,0x00,0x12,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x00,0x3E,0x00,0x16,0x05,0xDC,0x01,0x56,0x00,0x56,0x00,0x15,0x0E,0xD8};

int vddmin_flag = true;
int light_flag = 1;
struct CWMCU_T {
    struct i2c_client *client;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    struct regulator *vdd_sensors;  //zhujp2 add
    //struct regulator *vdd_ir;       //zhujp2 add 
    struct input_polled_dev *input_polled;
    struct input_dev *input;
    struct workqueue_struct *driver_wq;
    struct work_struct work;
    struct delayed_work delay_work;
    struct CWMCU_SENSORS_INFO sensors_info[HANDLE_ID_END][SENSORS_ID_END];
    SensorsInit_T   hw_info[DRIVER_ID_END];
    RegInformation *pReadRegInfo;
    RegInformation *pWriteRegInfo;
    u8 m_cReadRegCount;
    u8 m_cWriteRegCount;
    uint8_t initial_hw_config;
    
    int mcu_mode;
    uint8_t kernel_status;

    /* enable & batch list */
    uint32_t enabled_list[HANDLE_ID_END];
    uint32_t interrupt_status;
    uint8_t calibratordata[DRIVER_ID_END][30];
    uint8_t calibratorUpdate[DRIVER_ID_END];
    
    /* Mcu site enable list*/

    /* power status */
volatile    uint32_t power_on_list;

    /* Calibrator status */
    int cal_cmd;
    int cal_type;
    int cal_id;

    /* gpio */
    int irq_gpio;
    int wakeup_gpio;
    int reset_gpio;            //ZHUJP2 ADD 
    int boot_gpio;             //zhujp2 add
    int IRQ;

    uint32_t debug_log;
    
    int cmd;
    uint32_t addr;
    int len;
    int mcu_slave_addr;
    int firmwave_update_status;
    int cw_i2c_rw;  /* r = 0 , w = 1 */
    int cw_i2c_len;
    uint8_t cw_i2c_data[300];

    s32 iio_data[6];
    struct iio_dev *indio_dev;
    struct irq_work iio_irq_work;
    struct iio_trigger  *trig;
    atomic_t pseudo_irq_enable;

    struct class *sensor_class;
    struct device *sensor_dev;
    atomic_t delay;
    int supend_flag;
	//int p_flag;

    int wq_polling_time;
	
#ifdef CWMCU_MUTEX
    struct mutex mutex_lock;
    struct mutex mutex_lock_i2c;
    struct mutex mutex_wakeup_gpio;
#endif
    
    unsigned char loge_buff[QueueSystemInfoMsgSize*2];
    unsigned char loge_buff_count;
    
    unsigned char logw_buff[QueueWarningMsgSize*2];
    unsigned char logw_buff_count;
    
    uint8_t logcat_cmd;
    uint16_t ir_data_length;
    uint8_t ir_ReportData[2048];
    char source[4096];
    uint8_t irInputData[2048];
    uint32_t temp[2000];
	uint8_t temp_ReportData[2048];
};

static DEFINE_MUTEX(multi_thread);
static DEFINE_MUTEX(i2c_access_mutex);
//for geater than 32 bytes read

static void wakeup_pin_reset(void)
{
    gpio_set_value(GPIO_CW_MCU_WAKE_UP, 0);
    usleep_range(200, 200);
    gpio_set_value(GPIO_CW_MCU_WAKE_UP, 1);
	usleep_range(200, 200);
}

static int CWMCU_Object_read(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
    int ret;

    struct i2c_msg msgs[] = {
        {
            .addr   = sensor->client->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &reg_addr,  
        },
        {
            .addr   = sensor->client->addr,
            .flags  = I2C_M_RD,
            .len    = len,
            .buf    = data,  
        },
    };

    ret = i2c_transfer(sensor->client->adapter, msgs, 2);  
    return (ret == 2) ? len : ret;
}

static int CWMCU_reg_read(struct CWMCU_T *sensor, u8 reg_addr, u8 *data)
{
    RegInformation *pReadRegInfoInx = sensor->pReadRegInfo;
    int i;
    u8 cReadRegCount = sensor->m_cReadRegCount;
    int wRetVal = 0;

    if(pReadRegInfoInx == NULL || cReadRegCount == 0)
    {
        wRetVal = -1;
        return wRetVal;
    }

    for(i = 0; i < cReadRegCount; i++)
    {
        if(pReadRegInfoInx[i].cIndex == reg_addr)
            break;
    }
    if(i >= cReadRegCount)
    {
        wRetVal = -1;
    }
    else
    {
        if(pReadRegInfoInx[i].cObjLen != 0)
            wRetVal = CWMCU_Object_read(sensor, pReadRegInfoInx[i].cIndex, data, pReadRegInfoInx[i].cObjLen);
    }
    return wRetVal;
}

// Returns the number of read bytes on success 

static int CWMCU_I2C_R(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
    int rty = 0;
    int err = 0;
#ifdef CWMCU_MUTEX
    mutex_lock(&i2c_access_mutex);
#endif
retry:
    err = i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data);
    if(err <0){
        
        if(rty<1)
        {
            rty++; 
			wakeup_pin_reset();
            //usleep_range(200*(rty+1), 300*(rty+1));
            goto retry;
    }
        else
        {
            pr_err("%s:%s:(i2c read error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,err);
        }
    }
#ifdef CWMCU_MUTEX
    mutex_unlock(&i2c_access_mutex);
#endif
    return err;
}

// write format    1.slave address  2.data[0]  3.data[1] 4.data[2]
static int CWMCU_I2C_W(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
    int rty = 0;
    int err = 0;
#ifdef CWMCU_MUTEX
    mutex_lock(&i2c_access_mutex);
#endif
retry:
    err = i2c_smbus_write_i2c_block_data(sensor->client, reg_addr, len, data);
    if(err <0){
        if(rty<1)
        {
            rty++; 
			wakeup_pin_reset();
            //usleep_range(200*(rty+1), 300*(rty+1));
            goto retry;
        }
        else
        {
            pr_err("%s:%s:(i2c write error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,err);
    }
    }
#ifdef CWMCU_MUTEX
    mutex_unlock(&i2c_access_mutex);
#endif
    return err;
}
    

static int CWMCU_I2C_W_SERIAL(struct CWMCU_T *sensor,u8 *data, int len)
{
    int dummy;
	mutex_lock(&i2c_access_mutex);
    dummy = i2c_master_send(sensor->client, data, len);
    if (dummy < 0) {
        pr_err("%s:%s:(i2c write error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,dummy);
		mutex_unlock(&i2c_access_mutex);
        return dummy;
    }
	mutex_unlock(&i2c_access_mutex);
    return 0;
}

static int CWMCU_I2C_R_SERIAL(struct CWMCU_T *sensor,u8 *data, int len)
{
    int dummy;
	mutex_lock(&i2c_access_mutex);
    dummy = i2c_master_recv(sensor->client, data, len);
    if (dummy < 0) {
        pr_err("%s:%s:(i2c read error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,dummy);
		mutex_unlock(&i2c_access_mutex);
        return dummy;
    }
	mutex_unlock(&i2c_access_mutex);
    return 0;
}


static int CWMCU_write_i2c_multiple(struct CWMCU_T *sensor, u8 reg_addr, const u8 *data, int len)
{
    int dummy;
    unsigned char buffer[len+1];
    buffer[0] = reg_addr;
    
    memcpy(&buffer[1], data, len);
	mutex_lock(&i2c_access_mutex);
    dummy = i2c_master_send(sensor->client, buffer, len+1);
    if (dummy < 0) {
        printk("CWMCU i2c master send fail! error=%d\n", dummy);
		mutex_unlock(&i2c_access_mutex);
        return dummy;
    }
	mutex_unlock(&i2c_access_mutex);
    return 0;
}

static int CWMCU_read_i2c_multiple(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, int len)
{
    int dummy=0;
    int rty=0;
	
    mutex_lock(&i2c_access_mutex);
retry1:		
    dummy = i2c_master_send(sensor->client, &reg_addr, 1);
	if(dummy <0){
        if(rty<2)
        {
            rty++; 
			wakeup_pin_reset();
            goto retry1;
        }
        else
        {
             printk("CWMCU i2c master send fail! error=%d\n", dummy);
        }
    }
	

 retry2:	
    dummy = i2c_master_recv(sensor->client, data, len);
	if(dummy <0){
        if(rty<2)
        {
            rty++; 
			wakeup_pin_reset();
            goto retry2;
        }
        else
        {
             printk("CWMCU i2c master recv fail! error=%d\n", dummy);
        }
    }	
	

	
    mutex_unlock(&i2c_access_mutex);
    return 0;
}
static int cw_send_event(struct CWMCU_T *sensor, u8 handle, u8 id, u8 *data)
{
    u8 event[21];/* Sensor HAL uses fixed 21 bytes */
    if (id == CWMCU_NODATA)
        return FAIL;

    event[0] = handle;
    event[1] = id;
    memcpy(&event[2], data, 19);
    if( (id==3) && light_flag ==0) {    
		printk("level=0 and disable light sensor\n");
		return 0;
	}

    if (sensor->debug_log )
        printk("%s: id%d,data:%d,%d,%d\n",
          __func__,id,data[0],data[1],data[2]);

     if (id ==4) {
            printk("%s: id%d, psensor status:%d,%d,%d\n",__func__,id,data[0],data[1],data[2]);
     }
	 /*
	     if (data[0] ==1 && sensor->p_flag == 1){
			 printk("SensorHub %s: PROX Status is far data=%d\n", __func__, data[0]);
		     sensor->p_flag = 0;
		}

        if (data[0] ==0 && sensor->p_flag == 0) {
            printk("SensorHub %s: PROX Status is near data=%d\n", __func__, data[0]);
		    sensor->p_flag = 1;
		}
	 } */
	 if (id == 20)
		printk("%s: id%d,stepcounter data:%d,%d,%d\n",__func__,id,data[0],data[1],data[2]);

    if (sensor->indio_dev->active_scan_mask &&
        (!bitmap_empty(sensor->indio_dev->active_scan_mask,
               sensor->indio_dev->masklength))) {
            iio_push_to_buffers(sensor->indio_dev, event);
        return 0;
    } else if (sensor->indio_dev->active_scan_mask == NULL)
        printk("%s: active_scan_mask = NULL, event might be missing\n",
          __func__);
    return -EIO;
}

static void power_pin_sw(struct CWMCU_T *sensor,SWITCH_POWER_ID id, int onoff)
{
	int value = 0;
	mutex_lock(&i2c_access_mutex);
	value = gpio_get_value(GPIO_CW_MCU_WAKE_UP);
    if (onoff) {
		sensor->power_on_list |= ((uint32_t)(1) << id);
		if(value == 0)
        gpio_set_value(GPIO_CW_MCU_WAKE_UP, onoff);
		if ((sensor->power_on_list == 0) || value == 0)
		{
			usleep_range(200, 200);
		}   
	}
	else
	{
		sensor->power_on_list &= ~(1 << id);
		if ((sensor->power_on_list == 0) && value)
		{
            gpio_set_value(GPIO_CW_MCU_WAKE_UP, onoff);
			usleep_range(200, 200);
        }
    }
	mutex_unlock(&i2c_access_mutex);
}

static void cwmcu_kernel_status(struct CWMCU_T *sensor,uint8_t status)
{
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return ;
    }
    sensor->kernel_status = status;
    CWMCU_I2C_W(sensor, RegMapW_SetHostStatus, &sensor->kernel_status, 1);
}

static int check_enable_list(struct CWMCU_T *sensor)
{
    int i = 0,j=0;
    int count = 0;
    int handle = 0;
    uint8_t data[10];
    int error_msg = 0;
    uint32_t enabled_list[HANDLE_ID_END] = {0};
    uint32_t enabled_list_temp = 0;
	if (CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8) >= 0)
	{
    enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
    enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
    enabled_list[InternalHandle] = 0;
        
    if((enabled_list[NonWakeUpHandle] != sensor->enabled_list[NonWakeUpHandle]) 
            || (enabled_list[WakeUpHandle] != sensor->enabled_list[WakeUpHandle]))
    {
			SH_LOG("Enable List Check AP0:%d,MCU0:%d;AP1:%d,MCU1:%d\n",
        sensor->enabled_list[NonWakeUpHandle],enabled_list[NonWakeUpHandle],
        sensor->enabled_list[WakeUpHandle],enabled_list[WakeUpHandle]);

        for(j = 0; j < InternalHandle; j++)
        {
            handle = j;
            enabled_list_temp = sensor->enabled_list[handle]^enabled_list[handle];
            for (i = 0; i < SENSORS_ID_END; i++)
            {
                if (enabled_list_temp & (1<<i))
                {
                    data[0] = handle;
                    data[1] = i;
                    if (sensor->sensors_info[handle][i].en)
                    {
                        sensor->sensors_info[handle][i].rate = (sensor->sensors_info[handle][i].rate ==0)?200:sensor->sensors_info[handle][i].rate;
                        data[2] = sensor->sensors_info[handle][i].rate;
                        data[3] = (uint8_t)sensor->sensors_info[handle][i].timeout;
                        data[4] = (uint8_t)(sensor->sensors_info[handle][i].timeout >>8);
                        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
                    }
                    else
                    {
                        data[2] = 0;
                        data[3] = 0;
                        data[4] = 0;
                        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetDisable, data, 5);
                    }
                    if (error_msg < 0)
                        SH_ERR("I2c Write Fail;%d,%d\n", handle, i);
                    count++;
                    if(count >15)
                    {
                        count = 0;
                        msleep(20);
                    }
                }
            }
        }
		}
        else {
			SH_LOG("enabled_list is the same sensor->enabled_list\n");
        }
	}
    else
    {
        SH_ERR("read RegMapR_GetHostEnableList failed!\n");
    }
    return 0;
}

static int cwmcu_read_buff(struct CWMCU_T *sensor , u8 handle)
{
    uint8_t count_reg;
    uint8_t data_reg;
    uint8_t data[24] = {0};
    uint16_t count = 0;
    int i = 0;
    vddmin_flag = false;
    if (handle == NonWakeUpHandle)
    {
        count_reg = RegMapR_StreamCount;
        data_reg = RegMapR_StreamEvent;
    }
    else if (handle == WakeUpHandle)
    {
        count_reg = RegMapR_BatchCount;
        data_reg = RegMapR_BatchEvent;
    }
    else
    {
        return FAIL;
    }

    if (CWMCU_I2C_R(sensor, count_reg, data, 2) >= 0) 
    {
        count = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }
    else 
    {
        SH_ERR("check count failed)\n");
        return FAIL;
    }
	
    if((data[0] ==0xFF) && (data[1] ==0xFF))
        return NO_ERROR;
    
    for (i = 0; i < count; i++)
    {
        if (CWMCU_I2C_R(sensor, data_reg, data, 9) >= 0)
        {
            cw_send_event(sensor,handle,data[0],&data[1]);
        }
    }
    vddmin_flag = true;
	return NO_ERROR;
}

static int cwmcu_read_gesture(struct CWMCU_T *sensor )
{
    uint8_t data[24] = {0};
    uint8_t count = 0;
    int i = 0;

    if (CWMCU_I2C_R(sensor, RegMapR_GestureCount, &count, 1) < 0)
    {
        SH_ERR("check count failed)\n");
            return FAIL;
    }
    if(count ==0xFF)
        return NO_ERROR;        

    for (i = 0; i < count; i++)
    {
        if (CWMCU_I2C_R(sensor, RegMapR_GestureEvent, data, 9) >= 0)
        {
                cw_send_event(sensor,NonWakeUpHandle,data[0],&data[1]);
        }
    }

    return NO_ERROR;
}

#define QueueSystemInfoMsgBuffSize      QueueSystemInfoMsgSize*5

static void parser_mcu_info(char *data)
{
    static unsigned char loge_bufftemp[QueueSystemInfoMsgBuffSize];
    static int buff_counttemp = 0;
    int i;

    for (i = 0; i < QueueSystemInfoMsgSize; i++)
    {
        loge_bufftemp[buff_counttemp] = (unsigned char)data[i];
        buff_counttemp++;
        if (data[i] == '\n' || (buff_counttemp >=QueueSystemInfoMsgBuffSize))
        {
            SH_LOG("%s:%s", "MSG",loge_bufftemp);
            memset(loge_bufftemp,0x00,QueueSystemInfoMsgBuffSize);
            buff_counttemp = 0;
        }
    }
}

static void read_mcu_info(struct CWMCU_T *sensor)
{
    uint8_t data[40];
    uint16_t count = 0;
    int i = 0;

    SH_FUN();
    if (CWMCU_I2C_R(sensor, RegMapR_SystemInfoMsgCount, data, 1) >= 0)
    {
        count = (uint16_t)data[0];
    }
    else
    {
		SH_ERR("check count fail!\n");
		return;
    }
	
    if(count ==0xFF)
        return ;

    for (i = 0; i < count; i++)
    {
        if (CWMCU_I2C_R(sensor, RegMapR_SystemInfoMsgEvent, data, 30) >= 0)
        {
            parser_mcu_info(data);
        }
    }

}

static int CWMCU_POLLING(struct CWMCU_T *sensor)
{
	//SH_LOG("In");
	
	if (sensor->debug_log & (1<<D_DELAY_WQ)) 
		SH_LOG("Polling: debug_log =>0x%x\n", (int)sensor->enabled_list[NonWakeUpHandle]);

	if (sensor->enabled_list[NonWakeUpHandle])
	{
		cwmcu_read_buff(sensor, NonWakeUpHandle);
		cwmcu_read_buff(sensor, WakeUpHandle);
	}
	
	//SH_LOG("Out");
	
    return 0;
}

/*==========sysfs node=====================*/
static int cwmcu_find_mindelay(struct CWMCU_T *sensor, int handle)
{
	int i;
	int min_delay = 60;
	for (i = 0; i < SENSORS_ID_END; i++) {	
		if(sensor->sensors_info[handle][i].en
				&& (sensor->sensors_info[handle][i].rate >= 10)
				&& (sensor->sensors_info[handle][i].rate < min_delay)
		  )
		{
			min_delay = sensor->sensors_info[handle][i].rate;
		}
	}
    min_delay = (min_delay<=10)? 10: min_delay;
	return min_delay;
}

static ssize_t active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int enabled = 0;
    int id = 0;
    int handle = 0;
    int error_msg = 0;
    uint8_t data[10];

    SH_FUN();
	
    
    sscanf(buf, "%d %d %d\n", &handle, &id, &enabled);

    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
    
    sensor->sensors_info[handle][id].en = enabled;
    data[0] = handle;
    data[1] = id;
	
    if (enabled)
    {
        sensor->enabled_list[handle] |= 1<<id;
        data[2] = (sensor->sensors_info[handle][id].rate ==0)?200:sensor->sensors_info[handle][id].rate;
        data[3] = (uint8_t)sensor->sensors_info[handle][id].timeout;
        data[4] = (uint8_t)(sensor->sensors_info[handle][id].timeout >>8);
        if (CW_BOOT == sensor->mcu_mode)
        {
         SH_ERR("mcu_mode = CW_BOOT\n");
		 return FAIL;
	    }
        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
    }
    else
    {
        sensor->enabled_list[handle] &= ~(1<<id);
        sensor->sensors_info[handle][id].rate = 0;
        sensor->sensors_info[handle][id].timeout = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
		
		if (CW_BOOT == sensor->mcu_mode)
        {
         SH_ERR("mcu_mode = CW_BOOT\n");
		 return FAIL;
	    }
        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetDisable, data, 5);
    }

    if (error_msg < 0)
        SH_ERR("I2c Write Fail\n");

    msleep(5);
	check_enable_list(sensor);
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);

    if (NonWakeUpHandle == handle)
    {
        SH_LOG("NonWakeUpHandle == handle\n");
        sensor->wq_polling_time = cwmcu_find_mindelay(sensor, NonWakeUpHandle);
        if (sensor->wq_polling_time != atomic_read(&sensor->delay))
		{
            SH_LOG("sensor->wq_polling_time != atomic_read(&sensor->delay)\n");
            cancel_delayed_work_sync(&sensor->delay_work);
        	if (sensor->enabled_list[NonWakeUpHandle])
        	{
                SH_LOG("sensor->enabled_list[NonWakeUpHandle] == 1\n");
                atomic_set(&sensor->delay, sensor->wq_polling_time);
            	queue_delayed_work(sensor->driver_wq, &sensor->delay_work,
            	msecs_to_jiffies(atomic_read(&sensor->delay)));
        	}
        	else
			{
        		atomic_set(&sensor->delay, CWMCU_POLL_MAX);
			}
    	}
    }
	
	if (sensor->debug_log) 
		SH_LOG("%d,%d,%d,%d,%d\n", handle, id, enabled, (int)sensor->sensors_info[handle][id].rate, (int)sensor->sensors_info[handle][id].timeout);

	return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[10] = {0};
    uint32_t enabled_list[2] ={0, 0};

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode = CW_BOOT\n");
        return sprintf(buf, "In Boot Mode\n");
    }

    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
    if (CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8) >= 0)
    {
        enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
        enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
        if (sensor->debug_log) 
            SH_LOG("MCU En Status:%d,%d\n", enabled_list[NonWakeUpHandle], enabled_list[WakeUpHandle]);
    }
    else
    {
        SH_ERR("check MCU En Status failed!\n");
    }
    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);

    return sprintf(buf, "%d %d %d %d\n", sensor->enabled_list[NonWakeUpHandle],
            sensor->enabled_list[WakeUpHandle], enabled_list[NonWakeUpHandle],enabled_list[WakeUpHandle]);
}

static ssize_t batch_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint32_t id = 0;
    uint32_t handle = 0;    
    uint32_t mode = -1;
    uint32_t rate = 0;
    uint32_t timeout = 0;
    int error_msg = 0;
    uint8_t data[5];


	sscanf(buf, "%d %d %d %d %d\n", &handle, &id, &mode, &rate, &timeout);
	
	if (mode == 0)
	{
		sensor->sensors_info[handle][id].rate = (uint8_t)rate;
		sensor->sensors_info[handle][id].timeout = (uint16_t)timeout;
		data[0] = handle;
		data[1] = id;
		data[2] = sensor->sensors_info[handle][id].rate;
		data[3] = (uint8_t)sensor->sensors_info[handle][id].timeout;
		data[4] = (uint8_t)(sensor->sensors_info[handle][id].timeout >> 8);
		
		if (CW_BOOT == sensor->mcu_mode)
        {
         SH_ERR("mcu_mode = CW_BOOT\n");
		 return FAIL;
        }
		
        if(sensor->sensors_info[handle][id].en)
        {
        	power_pin_sw(sensor,SWITCH_POWER_BATCH, 1);
            error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
        	power_pin_sw(sensor,SWITCH_POWER_BATCH, 0);
			
        if (error_msg < 0)
                SH_ERR("Write Fail:id:%d, mode:%d, rate:%d, timeout:%d)\n", id, mode, rate, timeout);
        }
    }

    if (sensor->debug_log) 
        SH_LOG("id:%d, mode:%d, rate:%d, timeout:%d)\n", id, mode, rate, timeout);

    return count;
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int id = 0;
    int error_msg = 0;
    int handle = 0; 
    uint8_t data[2];

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    sscanf(buf, "%d %d\n", &handle, &id);
    SH_LOG("flush:id:%d\n", id);
    data[0] = (uint8_t)handle;
    data[1] = (uint8_t)id;
	
	power_pin_sw(sensor,SWITCH_POWER_FLUSH, 1);
    error_msg = CWMCU_I2C_W(sensor, RegMapW_SetFlush, data, 2);
    power_pin_sw(sensor,SWITCH_POWER_FLUSH, 0);

    return count;
}

static int CWMCU_Write_Mcu_Memory(struct CWMCU_T *sensor,const char *buf)
{
    uint8_t WriteMemoryCommand[2];
    uint8_t data[300];
    uint8_t received[10];
    uint8_t XOR = 0;
    uint16_t i = 0;

    WriteMemoryCommand[0] = 0x31;
    WriteMemoryCommand[1] = 0xCE;
    if (CWMCU_I2C_W_SERIAL(sensor,(uint8_t *)WriteMemoryCommand, 2) < 0)
    {
        return -EAGAIN;
        }
    
    if (CWMCU_I2C_R_SERIAL(sensor,(uint8_t *)received, 1) < 0)
    {
        return -EAGAIN;
        }

    if (received[0] != ACK)
    {
        return -EAGAIN;
        }
    
    data[0] = (uint8_t) (sensor->addr >> 24);
    data[1] = (uint8_t) (sensor->addr >> 16);
    data[2] = (uint8_t) (sensor->addr >> 8);
    data[3] = (uint8_t) sensor->addr;
    data[4] = data[0] ^ data[1] ^ data[2] ^ data[3];
    if (CWMCU_I2C_W_SERIAL(sensor,(uint8_t *)data, 5) < 0)
    {
        return -EAGAIN;
        }

    if (CWMCU_I2C_R_SERIAL(sensor,(uint8_t *)received, 1) < 0)
    {
        return -EAGAIN;
    }

    if (received[0] != ACK)
    {
        return -EAGAIN;
    }

    data[0] = sensor->len - 1;
    XOR = sensor->len - 1;
    for (i = 0; i < sensor->len; i++)
    {
        data[i+1] = buf[i];
        XOR ^= buf[i];
    }
    data[sensor->len+1] = XOR;

    if (CWMCU_I2C_W_SERIAL(sensor,(uint8_t *)data, (sensor->len + 2)) < 0)
    {
        return -EAGAIN;
    }

    return 0;
}

static int set_calib_cmd(struct CWMCU_T *sensor, uint8_t cmd, uint8_t id, uint8_t type)
{
    uint8_t data[4];
    int err;

    if (CW_BOOT == sensor->mcu_mode) 
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return -1;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    data[0] = cmd;
    data[1] = id;
    data[2] = type;
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorCmd, data, 4);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);

    return err;
}

/*
    sensors default calibrator flow:
        sensors_calib_start(sensors, id);
        do{
            sensors_calib_status(sensors, id,&status);
        }while(status ==CALIB_STATUS_INPROCESS)
        if(status ==CALIB_STATUS_PASS)
            sensors_calib_data_read(sensors, id,data);
        save data    
*/

static int sensors_calib_start(struct CWMCU_T *sensor, uint8_t id)
{
    int err;
	
    err = set_calib_cmd(sensor, CALIB_EN, id, CALIB_TYPE_DEFAULT);
	
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    err = set_calib_cmd(sensor, CALIB_CHECK_STATUS, id, CALIB_TYPE_NON);
	
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    return err;
}

static int sensors_calib_status(struct CWMCU_T *sensor, uint8_t id, int *status)
{
    int err;
    uint8_t i2c_data[31] = {0};

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	
    if (err < 0)
    {
        SH_ERR("I2c Read Fail!\n");
        return I2C_FAIL;
    }
	
    status[0] = (int)((int8_t)i2c_data[0]);

    return NO_ERROR;
}

static int sensors_calib_data_read(struct CWMCU_T *sensor, uint8_t id, uint8_t *data)
{
    int err;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_READ, id, CALIB_TYPE_NON);
	
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	
    if (err < 0)
    {
        SH_ERR("I2c Read Fail!\n");
        return err;
    }

    return NO_ERROR;
}

static int sensors_calib_data_write(struct CWMCU_T *sensor, uint8_t id, uint8_t *data)
{
    int err;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_WRITE, id, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

	printk("P Calib data %d %d %d %d\n",data[0],data[1],data[2],data[3]);
	
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    return NO_ERROR;
}

static int proximity_calib_en(struct CWMCU_T *sensor, int en)
{
    int err;
    
    SH_LOG("en=%d\n", en);
	
    if(en)
        err = set_calib_cmd(sensor, CALIB_EN, PROXIMITY, CALIB_TYPE_SENSORS_ENABLE);
    else
        err = set_calib_cmd(sensor, CALIB_EN, PROXIMITY, CALIB_TYPE_SENSORS_DISABLE);

    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }
    
    return NO_ERROR;
}

/*
    FUN: proximity_calib_data
    |data[0]: Proximity sensors raw data
    |data[1] is Hight threshold to check sensors is near 
    |data[2] is low threshold to check sensors is far 
*/

static int proximity_calib_data(struct CWMCU_T *sensor, int *data)
{
    int err;
    uint8_t i2c_data[31] = {0};
    int *ptr;
    ptr = (int *)i2c_data;
    
    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_READ, PROXIMITY, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("set_calib_cmd Fail!\n");
        return err;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
        SH_ERR("I2c Read Fail!\n");
        return I2C_FAIL;
    }

    data[0] = ptr[3];
    data[1] = ptr[1];
    data[2] = ptr[2];

    SH_LOG("raw:%u, close:%u, far:%u\n",i2c_data[1], i2c_data[2], i2c_data[3]);
	
    return NO_ERROR;
}

static int proximity_set_threshold(struct CWMCU_T *sensor, int near_th, int far_th)
{
    int err;
    uint8_t i2c_data[31] = {0};
    int *ptr;
    ptr = (int *)i2c_data;
    
    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_WRITE, PROXIMITY, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("set_calib_cmd Fail!\n");
        return err;
    }

    ptr[0] = 0;
    ptr[1] = near_th;
    ptr[2] = far_th;
    
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, i2c_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return -1;
    }

    SH_LOG("close:%d, far:%d\n", near_th, far_th);
	
    return NO_ERROR;
}

static ssize_t set_firmware_update_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    u8 data[300] = {0};
    int i = 0;
    int status = 0;
    int proximity_data[3] = {0};
    u8 cReadRegCount = sensor->m_cReadRegCount;
    u8 cWriteRegCount = sensor->m_cWriteRegCount;
    RegInformation *pReadRegInfoInx = sensor->pReadRegInfo;
    RegInformation *pWriteRegInfoInx = sensor->pWriteRegInfo;
    
    sscanf(buf, "%d %d %d\n", &sensor->cmd, &sensor->addr, &sensor->len);
    
    SH_LOG("cmd=%d addr=%d len=%d\n", sensor->cmd, sensor->addr, sensor->len);

    power_pin_sw(sensor,SWITCH_POWER_FIRMWARE_COMMAND, 1);

    switch (sensor->cmd) {
    case CHANGE_TO_BOOTLOADER_MODE:
            printk("%s:%s:(CHANGE_TO_BOOTLOADER_MODE)\n",LOG_TAG_KERNEL ,__FUNCTION__);

            gpio_direction_output(GPIO_CW_MCU_RESET, 1);
            gpio_set_value(GPIO_CW_MCU_BOOT, 1);
            msleep(500);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
            msleep(500);
            gpio_set_value(GPIO_CW_MCU_RESET, 0);
            msleep(500);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
            msleep(1000);

            sensor->mcu_mode = CW_BOOT;
            sensor->mcu_slave_addr = sensor->client->addr;
            sensor->client->addr = 0x72 >> 1;
            break;

    case CHANGE_TO_NORMAL_MODE:
            printk("%s:%s:(CHANGE_TO_NORMAL_MODE)\n",LOG_TAG_KERNEL ,__FUNCTION__);

            sensor->firmwave_update_status = 1;
            sensor->client->addr = 0x74 >> 1;

            
            gpio_set_value(GPIO_CW_MCU_BOOT, 0);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
            msleep(500);
            gpio_set_value(GPIO_CW_MCU_RESET, 0);
            msleep(500);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
            msleep(1000);

            gpio_direction_input(GPIO_CW_MCU_RESET);

            sensor->mcu_mode = CW_NORMAL;
            break;

    case CHECK_FIRMWAVE_VERSION:
            if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, data, 4) >= 0) {
                printk("%s:%s:(CHECK_FIRMWAVE_VERSION:%u,%u,%u,%u)\n",LOG_TAG_KERNEL ,__FUNCTION__, data[0],data[1],data[2],data[3]);
            }
            break;

    case GET_FWPROJECT_ID:
            if (CWMCU_reg_read(sensor, RegMapR_GetProjectID, data) >= 0) {
                printk("%s:%s:(PROJECT ID:%s) \n",LOG_TAG_KERNEL ,__FUNCTION__, data);
            }
            break;

    case SHOW_THE_REG_INFO:
            if (pWriteRegInfoInx != NULL && pReadRegInfoInx != NULL) {
               printk("(number of read reg:%u number of write reg:%u) \n",cReadRegCount ,cWriteRegCount);
               printk("--------------------READ REGISTER INFORMATION------------------------\n");
               for(i =0; i < cReadRegCount; i++)
               {
                   printk("(read tag number:%u and lengh:%u) \n",pReadRegInfoInx->cIndex,pReadRegInfoInx->cObjLen);
                   pReadRegInfoInx++;
               }

               printk("--------------------WRITE REGISTER INFORMATION-----------------------\n");
               for(i =0; i < cWriteRegCount; i++)
               {
                   printk("(write tag number:%u and lengh:%u) \n",pWriteRegInfoInx->cIndex ,pWriteRegInfoInx->cObjLen);
                   pWriteRegInfoInx++;
               }
            }
            break;

    case SET_DEBUG_LOG:
                    if(sensor->len)
                        sensor->debug_log  |= (1<< sensor->addr);
                    else
                        sensor->debug_log  &= ~(1<< sensor->addr);
            printk("%s:%s:(SET_DEBUG_LOG%u)\n",LOG_TAG_KERNEL ,__FUNCTION__,sensor->debug_log);
            break;
    case SET_SYSTEM_COMMAND:
            data[0] = sensor->addr;
            data[1] = sensor->len;
            CWMCU_I2C_W(sensor, RegMapW_SetSystemCommand, data, 2);
            printk("%s:%s:(SET_SYSTEM_COMMAND)\n",LOG_TAG_KERNEL ,__FUNCTION__);
            break;
    case GET_SYSTEM_TIMESTAMP:
            if (CWMCU_I2C_R(sensor, RegMapR_GetSystemTimestamp, data, 4) >= 0) {
                printk("%s:%s:(Timestamp:%u)\n",LOG_TAG_KERNEL ,__FUNCTION__,(((uint32_t)data[3])<<24) |(((uint32_t)data[2])<<16) |(((uint32_t)data[1])<<8) | ((uint32_t)data[0]));
            }
            break;
    case SET_HW_INITIAL_CONFIG_FLAG:
            sensor->initial_hw_config = sensor->addr;
            break;
    case SET_SENSORS_POSITION:
            data[0] = sensor->addr;
            data[1] = sensor->len;
            CWMCU_I2C_W(sensor, RegMapW_SetSensorAxisReference, data, 2);
            break;
    case CMD_CALIBRATOR_START:
        sensors_calib_start(sensor,sensor->addr);
        break;
    case CMD_CALIBRATOR_STATUS:
        sensors_calib_status(sensor,sensor->addr,&status);
        break;
    case CMD_CALIBRATOR_READ:
        sensors_calib_data_read(sensor,sensor->addr,sensor->cw_i2c_data);
        break;
    case CMD_CALIBRATOR_WRITE:
        sensors_calib_data_write(sensor,sensor->addr,sensor->cw_i2c_data);
        break;
    case CMD_PROXIMITY_EN:
        proximity_calib_en(sensor,sensor->addr);
        break;
    case CMD_PROXIMITY_DATA:
        proximity_calib_data(sensor,proximity_data);
            printk("%s:%s:(Proximity data:%d,%d,%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,proximity_data[0],proximity_data[1],proximity_data[2]);
        break;
    case CMD_PROXIMITY_TH:
        proximity_set_threshold(sensor,sensor->addr,sensor->len);
        printk("%s:%s:(Proximity th:%d,%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,sensor->addr,sensor->len);
            break;
    	default:
			break;
    }
    power_pin_sw(sensor,SWITCH_POWER_FIRMWARE_COMMAND, 0);
	
    return count;
}

static ssize_t set_firmware_update_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
	
    SH_LOG("%s\n", buf);
    sensor->firmwave_update_status = 1;
    sensor->firmwave_update_status = CWMCU_Write_Mcu_Memory(sensor,buf);
	
    return count;
}

static ssize_t get_firmware_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    SH_LOG("firmwave_update_status = %d\n", sensor->firmwave_update_status);
    return sprintf(buf, "%d\n", sensor->firmwave_update_status);
}

static ssize_t set_firmware_update_i2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int intsize = sizeof(int);

   // SH_FUN();
    memcpy(&sensor->cw_i2c_rw, buf, intsize);
    memcpy(&sensor->cw_i2c_len, &buf[4], intsize);
    memcpy(sensor->cw_i2c_data, &buf[8], sensor->cw_i2c_len);
	
    return count;
}

static ssize_t get_firmware_update_i2(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int status = 0;

   // SH_FUN();
    if (sensor->cw_i2c_rw)
    {
        if (CWMCU_I2C_W_SERIAL(sensor,sensor->cw_i2c_data, sensor->cw_i2c_len) < 0)
        {
            status = -1;
        }
        memcpy(buf, &status, sizeof(int));
        return 4;
    }
    else
    {
        if (CWMCU_I2C_R_SERIAL(sensor,sensor->cw_i2c_data, sensor->cw_i2c_len) < 0)
        {
            status = -1;
            memcpy(buf, &status, sizeof(int));
            return 4;
        }
        memcpy(buf, &status, sizeof(int));
        memcpy(&buf[4], sensor->cw_i2c_data, sensor->cw_i2c_len);
        return 4+sensor->cw_i2c_len;
    }
	
    return  0;
}

static ssize_t mcu_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", sensor->mcu_mode);
}

static ssize_t mcu_model_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int mode = 0;
	
    sscanf(buf, "%d\n", &mode);
    sensor->mcu_mode = mode;
	
    return count;
}

static ssize_t set_calibrator_cmd(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int err;

    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    sscanf(buf, "%d %d %d\n", &sensor->cal_cmd, &sensor->cal_id, &sensor->cal_type);
    err = set_calib_cmd(sensor, sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
    
	if (sensor->debug_log) 
        SH_LOG("cmd:%d,id:%d,type:%d\n", sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");

    return count;
}

static ssize_t get_calibrator_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    return sprintf(buf, "Cmd:%d,Id:%d,Type:%d\n", sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
}

static ssize_t get_calibrator_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t Cal_data[31] = {0};
    int err;
    
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, Cal_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	
    if(sensor->cal_cmd == CALIB_DATA_READ && err >=0){
        memcpy(sensor->calibratordata[sensor->cal_id],Cal_data,30);
        sensor->calibratorUpdate[sensor->cal_id] = 1;
    }
	
	printk("P Calib data %d %d %d %d\n",sensor->calibratordata[4][0],sensor->calibratordata[4][1],sensor->calibratordata[4][2],sensor->calibratordata[4][3]);
    
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", 
        err,
        Cal_data[0], Cal_data[1], Cal_data[2],
        Cal_data[3], Cal_data[4], Cal_data[5], Cal_data[6], Cal_data[7], Cal_data[8], Cal_data[9], Cal_data[10], Cal_data[11], Cal_data[12],
        Cal_data[13], Cal_data[14], Cal_data[15], Cal_data[16], Cal_data[17], Cal_data[18], Cal_data[19], Cal_data[20], Cal_data[21], Cal_data[22],
        Cal_data[23], Cal_data[24], Cal_data[25], Cal_data[26], Cal_data[27], Cal_data[28], Cal_data[29]);
}

static ssize_t set_calibrator_data(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[30];
    int temp[33] = {0};
    int i,err;

    if (sensor->mcu_mode == CW_BOOT) 
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
        &temp[0], &temp[1], &temp[2],
        &temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8], &temp[9], &temp[10], &temp[11], &temp[12],
        &temp[13], &temp[14], &temp[15], &temp[16], &temp[17], &temp[18], &temp[19], &temp[20], &temp[21], &temp[22],
        &temp[23], &temp[24], &temp[25], &temp[26], &temp[27], &temp[28], &temp[29]);

    for (i = 0 ; i < 30; i++)
        data[i] = (uint8_t)temp[i];

    if(sensor->cal_cmd == CALIB_DATA_WRITE){
        memcpy(sensor->calibratordata[sensor->cal_id],data,30);
        sensor->calibratorUpdate[sensor->cal_id] = 1;
    }
	
    printk("P Calib data %d %d %d %d\n",sensor->calibratordata[4][0],sensor->calibratordata[4][1],sensor->calibratordata[4][2],sensor->calibratordata[4][3]);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, data, 30);
	power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");

    return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[4];
    int16_t version = -1;

    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
    if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, data, 4) >= 0)
    {
        version = (int16_t)( ((uint16_t)data[1])<<8 | (uint16_t)data[0]);
        SH_LOG("CHECK_FIRMWAVE_VERSION : M:%u,D:%u,V:%u,SV:%u\n", data[3], data[2], data[1], data[0]);
    }
    else
    {
        SH_ERR("i2c read fail)\n");
    }
    power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);
	
    return sprintf(buf, "%d\n", version);
}

static ssize_t library_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[4];

    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    
    power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
    if (CWMCU_I2C_R(sensor, RegMapR_GetLibVersion, data, 4) >= 0)
    {
        SH_LOG("check_library_version:%u,%u,%u,%u\n", data[3], data[2], data[1], data[0]);
    }
    else
    {
        SH_ERR("i2c read fail)\n");
    }
    power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);
	
    return sprintf(buf, "%d %d %d %d\n", data[3], data[2], data[1], data[0]);
}

static ssize_t timestamp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[4];
    uint32_t *ptr;
    int err;
    ptr = (uint32_t *)data;
    
    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_TIME, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetSystemTimestamp, data, 4);
    SH_LOG("Time:%u)\n", ptr[0]);
    power_pin_sw(sensor,SWITCH_POWER_TIME, 0);
    return sprintf(buf, "%d %u\n", err, ptr[0]);
}


static ssize_t set_sys_cmd(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[8];
    int temp[8] = {0};
    int i,err;

    if (sensor->mcu_mode == CW_BOOT) 
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    sscanf(buf, "%d %d %d %d %d %d %d %d\n",
        &temp[0], &temp[1], &temp[2],
        &temp[3], &temp[4], &temp[5], &temp[6], &temp[7]);

    for (i = 0 ; i < 8; i++)
        data[i] = (uint8_t)temp[i];

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_SetSystemCommand, data, 8);
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return count;
}

static void read_calib_info(struct CWMCU_T *sensor)
{
    uint8_t data[24] = {0};
    int status = 0;
    uint16_t *ptr;
    ptr = (uint16_t *)data;
    
    if(set_calib_cmd(sensor, CALIB_CHECK_STATUS, sensor->cal_id, sensor->cal_type)){
        SH_ERR("I2c Write Fail!\n");
        return;
    }

    if(sensors_calib_status(sensor,  sensor->cal_id, &status) >=0){
    SH_LOG("Calib id:%d:status:%d\n", sensor->cal_id , status);
        if (CALIB_STATUS_PASS == status)
        {
            ptr[0] =  (uint16_t)sensor->cal_id;
            cw_send_event(sensor,NonWakeUpHandle,CALIBRATOR_UPDATE,data);
        }
    }

    return ;

}
static ssize_t get_raw_data0(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetAccelerationRawData, data, 6);
    SH_LOG("RawData0:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}
static ssize_t get_raw_data1(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetMagneticRawData, data, 6);
    SH_LOG("RawData1:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}
static ssize_t get_raw_data2(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetGyroRawData, data, 6);
    SH_LOG("RawData2:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

static ssize_t get_raw_data3(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetLightRawData, data, 6);
    SH_LOG("RawData3:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

static ssize_t get_raw_data4(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetProximityRawData, data, 6);
    SH_LOG("RawData4:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

/***************zhujp2 add start 0806********************************/
static ssize_t enable_log_get(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->debug_log);
}



static ssize_t enable_log_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
	int err=0;
	sscanf(buf, "%d\n", &err);
	sensor->debug_log = err;
	return count;
}

/****************zhujp2 add end 0806*******************************/
static void reload_calib_data(struct CWMCU_T *sensor)
{
    int i;
    for(i = 0;i < DRIVER_ID_END ; i ++)
    {
        if(sensor->calibratorUpdate[i])
        {
            sensors_calib_data_write(sensor, i, sensor->calibratordata[i]);
            msleep(10);
        }
    }
}

static void cwmcu_reinit(struct CWMCU_T *sensor)
{
    reload_calib_data(sensor);
    check_enable_list(sensor);
    cwmcu_kernel_status(sensor,KERNEL_RESUND);
}

/*
//IR send
#define RegMapW_IR_SendInfo     0xF0 //6 bytes
#define RegMapW_IR_SendData     0xF1 //128 bytes, (1~n) times
#define RegMapW_IR_SendGo       0xF2 //1 byte: 0=>none, 1=>Go
#define RegMapR_IR_SendGoStatus 0xF3 //1 byte: 1=>transfer is completed, 0=>not ready
                                     
//IR Learning
#define RegMapR_IR_LearnEnable  0xF4
#define RegMapR_IR_LearnStatus  0xF5
#define RegMapR_IR_LearnGetInfo 0xF6
#define RegMapR_IR_LearnGetData 0xF7
*/

//static int ir_status = 0;
//static int ir_cmd_flag = 0;

//static uint8_t ir_ReportData[1000] = {0,};
//static char source[1000] = {0,};
//static uint8_t irInputData[1000] = {0,};

//mcu replay
static ssize_t irc_get(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int length = 0;
    int len = 0;
    int offset = 0;
    uint8_t ir_report[10];
    int i = 0;
    int loops = 0;

    printk("--CWMCU-- %s, buf addr=%p\n", __func__, buf);

    length = ((uint16_t)sensor->ir_ReportData[1] << 8) | (uint16_t)sensor->ir_ReportData[0];
    loops = (length/10)+1;
    offset += 2;

    for (i = 0; i < loops; i++) {
        memcpy(ir_report,sensor->ir_ReportData+offset,sizeof(uint8_t)*10);
        len += sprintf(buf + len, "%x %x %x %x %x %x %x %x %x %x ",
            ir_report[0],ir_report[1],ir_report[2],ir_report[3],ir_report[4],
            ir_report[5],ir_report[6],ir_report[7],ir_report[8],ir_report[9]);

        offset += 10;
    }

	printk("--CWMCU-- length=%d loops=%d offset=%d len=%d\n", length, loops, offset, len);
    printk("--CWMCU-- out buf=%s\n", buf);
	return len;
}


static ssize_t irc_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int i,j;
    int index = 0, loops = 0;

    char *pch;
    char *inBuf= sensor->source;

    uint8_t irPacket[128]={0,};
    uint8_t irSetData[20]={0,};
    uint8_t irGetData[20]={0,};
    
    uint8_t checksum = 0;
    uint8_t irstatus = 0;
    int offset = 0;
    
    printk("--CWMCU-- %s in buf addr=%p\n", __func__, buf);
    //0x53  0x54    0x11    A0  A1  B0  B1  0x00    0x00    0x00    0x00    0x00    0x00    0x00    0x00    C1  C2 =>17byte
    //0x53  0x54    0x22    Data0   Data1   ... ... Data(n)     n=A1A0  
    //0x53  0x54    0x33

    strcpy(sensor->source, buf);
    
    printk("--CWMCU-- %s , source: %s\n", __func__, sensor->source);
	memset(sensor->irInputData, 0, sizeof(uint8_t)*2000);  //zhujp2 add 0506
    
	while ((pch = strsep(&inBuf, ", ")) != NULL) {
        sscanf(pch, "%x", &sensor->temp[index]);
        printk("--CWMCU-- %s , sensor->temp[index]: %d, %x\n", __func__, sensor->temp[index], sensor->temp[index]);
        sensor->irInputData[index] = (uint8_t)sensor->temp[index];
        index++;
    }
    printk("--CWMCU-- %s , index: %d\n", __func__, index);
/*
    sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
        &irCmd[0],&irCmd[1],&irCmd[2],&irCmd[3],&irCmd[4],&irCmd[5],&irCmd[6],&irCmd[7],&irCmd[8],&irCmd[9],
        &irCmd[10],&irCmd[11],&irCmd[12],&irCmd[13],&irCmd[14],&irCmd[15],&irCmd[16]);

    printk("--CWMCU-- irCmd[]=> 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x \
                0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x\n",
                irCmd[0],irCmd[1],irCmd[2],irCmd[3],irCmd[4],irCmd[5],irCmd[6],irCmd[7],irCmd[8],irCmd[9],
                irCmd[10],irCmd[11],irCmd[12],irCmd[13],irCmd[14],irCmd[15],irCmd[16]);
*/
    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);

    if(sensor->irInputData[0]==0x53 && sensor->irInputData[1]==0x54 && sensor->irInputData[2]==0x11) {
        printk("--CWMCU-- %s , 0x11\n", __func__);
        memset(irSetData,0,sizeof(uint8_t)*8);
        irSetData[0] = sensor->irInputData[3];
        irSetData[1] = sensor->irInputData[4];
        irSetData[2] = sensor->irInputData[5];
        irSetData[3] = sensor->irInputData[6];
        irSetData[4] = sensor->irInputData[7];
        irSetData[5] = sensor->irInputData[8];
		irSetData[6] = sensor->irInputData[9];
		irSetData[7] = sensor->irInputData[10];
        
        sensor->ir_data_length = ((uint16_t)sensor->irInputData[4] << 8) | (uint16_t)sensor->irInputData[3];
        
        printk("--CWMCU-- %s lendth: %d\n", __func__, sensor->ir_data_length);

        if(CWMCU_I2C_W(sensor, RegMapW_IR_SendInfo, irSetData, 8) < 0) {
            printk("--CWMCU-- write IR info fail. [i2c]\n");
        }
    }

    if(sensor->irInputData[0]==0x53 && sensor->irInputData[1]==0x54 && sensor->irInputData[2]==0x22) {
        printk("--CWMCU-- %s , 0x22\n", __func__);
        memset(irPacket,0,sizeof(uint8_t)*128);
        loops = (sensor->ir_data_length / 124) + 1;
        printk("--CWMCU-- %s , loop:%d ir_data_length:%d\n", __func__, loops, sensor->ir_data_length);
		offset = 3;
        for(i = 0; i < loops; i++) {
            irPacket[0] = loops;
            irPacket[1] = i;        

            if( (i+1) == loops) {
                irPacket[2] = (sensor->ir_data_length%124); //transfer size
            } else {
                irPacket[2] = 124;
            }

            memcpy(irPacket+3,sensor->irInputData+offset,sizeof(uint8_t)*irPacket[2]);
			offset +=irPacket[2];
			checksum=0;
            for (j = 0; j < 124; j++) {
                checksum += irPacket[j+3];
            }
            irPacket[127] = checksum;

            if(CWMCU_write_i2c_multiple(sensor, RegMapW_IR_SendData, irPacket, 128) < 0) {
                printk("--CWMCU-- write IR data fail. [i2c]\n");
            }
            usleep(5000);
			printk("--CWMCU-- irPacket[0-4]:%d %d %d %d %d\n", irPacket[0], irPacket[1], irPacket[2], irPacket[3], irPacket[4]);
        }
    }


    if(sensor->irInputData[0]==0x53 && sensor->irInputData[1]==0x54 && sensor->irInputData[2]==0x33) {
        printk("--CWMCU-- %s , 0x33\n", __func__);
        memset(irPacket,0,sizeof(uint8_t)*128);
        irPacket[0] = 1;
        if(CWMCU_I2C_W(sensor, RegMapW_IR_SendGo, irPacket, 1) < 0) {
            printk("--CWMCU-- write IR data fail. [i2c]\n");
        }
    }

    //
    //for ir learning
    //

    //enable IR learning
    if(sensor->irInputData[0]==0x53 && sensor->irInputData[1]==0x54 && sensor->irInputData[2]==0x55) {
        printk("--CWMCU-- %s , 0x55\n", __func__);
        memset(irPacket,0,sizeof(uint8_t)*128);
        irPacket[0] = 1;
        if(CWMCU_I2C_W(sensor, RegMapR_IR_LearnEnable, irPacket, 1) < 0) {
            printk("--CWMCU-- write IR send learning start fail. [i2c]\n");
        }
    }

    //Get IR learning status
    if(sensor->irInputData[0]==0x53 && sensor->irInputData[1]==0x54 && sensor->irInputData[2]==0x66) {
        printk("--CWMCU-- %s , 0x66\n", __func__);
        if(CWMCU_I2C_R(sensor, RegMapR_IR_LearnStatus, irGetData, 1) < 0) {
            printk("--CWMCU-- write IR send learning start fail. [i2c]\n");
        }

        offset = 0;
        sensor->ir_data_length = 11;
        memset(sensor->ir_ReportData,0,sizeof(uint8_t)*1000);
        memcpy(sensor->ir_ReportData,&sensor->ir_data_length,2);
        offset += 2;
        
        memcpy(sensor->ir_ReportData+offset, sensor->irInputData,3);
        offset += 3;

        //IR learning reply
        if(irGetData[0] == 1) {
            //0x53  0x54    0x66    a0  a1  b0  b1  b2  b3  c0  c1
            if(CWMCU_I2C_R(sensor, RegMapR_IR_LearnGetInfo, irGetData, 8) < 0) {
                printk("--CWMCU-- write IR send learning start fail. [i2c]\n");
            }
            memcpy(sensor->ir_ReportData+offset,irGetData,sizeof(uint8_t)*8);
            
            printk("--CWMCU-- IR Get learning info ready [0-7]: %x %x %x %x %x %x %x %x\n",
                irGetData[0],irGetData[1],irGetData[2],irGetData[3],
                irGetData[4],irGetData[5],irGetData[6],irGetData[7]);
            
        }
        else {
		    printk("--CWMCU-- IR Get learning info not ready [0-7]: %x %x %x %x %x %x %x %x\n",
			irGetData[0],irGetData[1],irGetData[2],irGetData[3],
			irGetData[4],irGetData[5],irGetData[6],irGetData[7]);
            sensor->ir_ReportData[2] = 0;
/*
            memset(sensor->ir_ReportData,0,sizeof(uint8_t)*11);
            sensor->ir_ReportData[0] = 0x53;
            sensor->ir_ReportData[1] = 0x54;
*/
        }
        
        irstatus = irGetData[0];
    }


    if(sensor->irInputData[0]==0x53 && sensor->irInputData[1]==0x54 && sensor->irInputData[2]==0x77) {
        printk("--CWMCU-- %s , 0x77\n", __func__);
        offset = 0;
        sensor->ir_data_length = ((uint16_t)sensor->irInputData[4] << 8) | (uint16_t)sensor->irInputData[3];
        memcpy(sensor->ir_ReportData,&sensor->ir_data_length,2);
        offset += 2;

        loops = (sensor->ir_data_length / 124) + 1;
        printk("--CWMCU-- 77 %s , loop:%d\n", __func__, loops);
        memcpy(sensor->ir_ReportData+offset,sensor->irInputData,3);
        offset += 3;
        for (i = 0; i < loops; i++) {

            if(CWMCU_read_i2c_multiple(sensor, RegMapR_IR_LearnGetData, irPacket, 128) < 0) {
                printk("--CWMCU-- Read IR learning data fail. [i2c]\n");
            }

            memcpy(sensor->ir_ReportData+offset,irPacket+3,irPacket[2]);
            offset +=irPacket[2];
            //checksum
            checksum = 0;
            for (j = 0; j < 124; j++) {
                checksum += irPacket[j+3];
                //printk("--CWMCU-- checksum : %d, data[%d]: 0x%x\n", checksum, j+3, irPacket[j+3]);
            }
            if(checksum != irPacket[127]) {
                printk("--CWMCU-- learning data checksum fail.[%d] checksum:%u | %u\n",i,checksum,irPacket[127]);
            } else {
                printk("--CWMCU-- learning data checksum pass.[%d] checksum:%u | %u\n",i,checksum,irPacket[127]);
            }
            usleep(5000);
        }
    }
    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);
	
    return count;
}




/* PEEL IR */

static ssize_t peel_irc_get(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int length = 0;
    int len = 0;
    int offset = 0;
    uint8_t ir_report[10];
    int i = 0,j=0;
    int loops = 0;
	uint16_t checksum = 0;
	int remainder = 0;

	memset(sensor->temp_ReportData, 0, sizeof(uint8_t)*2048); 

	length = sensor->ir_data_length ;

       if(sensor->ir_ReportData[0] == 2 )  {
		printk("--CWMCU-- %s, read learning data buf addr=%p %x %x length=%d\n", __func__, buf,sensor->ir_ReportData[2],sensor->ir_ReportData[3],length);
       	} else if(sensor->ir_ReportData[0] == 1 )  {
       		length = 2;
       		printk("--CWMCU-- %s, read learning status buf addr=%p %x %x length=%d\n", __func__, buf,sensor->ir_ReportData[2],sensor->ir_ReportData[3],length);
       	}else {
       		printk("--CWMCU-- %s, addr=%p %x %x length=%d\n", __func__, buf,sensor->ir_ReportData[2] ,sensor->ir_ReportData[3],length);
       	}
	//kernel header takes 7 bytes already
	if(length >2041)  length = 2041;
	memcpy(sensor->temp_ReportData+7,sensor->ir_ReportData,sizeof(uint8_t)*length);
	
    for (i = 0; i < length; i++) {
           checksum += (uint16_t)sensor->temp_ReportData[i+7];
    }
		
	sensor->temp_ReportData[0]=0x4C;
	sensor->temp_ReportData[1]=0x01; 
	sensor->temp_ReportData[2]=(length>>8) & 0xFF;//sensor->ir_ReportData[2];
    sensor->temp_ReportData[3]=length & 0xFF;//sensor->ir_ReportData[3];
	sensor->temp_ReportData[4]=(uint8_t) ((checksum >> 8) & 0x00FF);
	sensor->temp_ReportData[5]=(uint8_t) (checksum & 0x00FF);
	sensor->temp_ReportData[6]=0x50;

	loops = ((length+7)/10);
	remainder =  (length+7)%10;
	for (j = 0; j < loops ; j++) {
		memcpy(ir_report,sensor->temp_ReportData+offset,sizeof(uint8_t)*10);
		len += sprintf(buf + len, "%x %x %x %x %x %x %x %x %x %x ",
		ir_report[0],ir_report[1],ir_report[2],ir_report[3],ir_report[4],
		ir_report[5],ir_report[6],ir_report[7],ir_report[8],ir_report[9]);
		offset += 10;
	}
	
	if(remainder) {
		memcpy(ir_report,sensor->temp_ReportData+offset,sizeof(uint8_t)*10);
		for (j =0 ; j < remainder;j++)
			len += sprintf(buf + len, "%x ", ir_report[j]);
		//printk("--CWMCU-- remainder=%d len=%d\n", remainder, len);
	}
	printk("--CWMCU-- length=%d loops=%d remainder=%d offset=%d len=%d\n", length, loops,remainder,offset, len);
	printk("--CWMCU-- out buf=%s\n", buf);
	
	return len;
}


static ssize_t peel_irc_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int i,j;
    int index = 0, loops = 0;

    char *pch;
    char *inBuf= sensor->source;

    uint8_t irPacket[128]={0,};
    uint8_t irGetData[20]={0,};
    
    uint8_t checksum = 0;
    uint8_t irstatus = 0;
    int offset = 0;
    
    printk("--CWMCU-- %s in buf addr=%p\n", __func__, buf);
    //0x53  0x54    0x11    A0  A1  B0  B1  0x00    0x00    0x00    0x00    0x00    0x00    0x00    0x00    C1  C2 =>17byte
    //0x53  0x54    0x22    Data0   Data1   ... ... Data(n)     n=A1A0  
    //0x53  0x54    0x33

    strcpy(sensor->source, buf);
    
    printk("--CWMCU-- %s , source: %s\n", __func__, sensor->source);
	memset(sensor->irInputData, 0, sizeof(uint8_t)*2048);  //zhujp2 add 0506
    
	while ((pch = strsep(&inBuf, ", ")) != NULL) {
        sscanf(pch, "%x", &sensor->temp[index]);
        printk("--CWMCU-- %s , sensor->temp[index]: %d, %x\n", __func__, sensor->temp[index], sensor->temp[index]);
        sensor->irInputData[index] = (uint8_t)sensor->temp[index];
        index++;
    }
    printk("--CWMCU-- %s , index: %d\n", __func__, index);

    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
	
	    //Get IR learning status
    if(sensor->irInputData[0]==0x50 && sensor->irInputData[6]==0x4C && sensor->irInputData[7]==0x01) {
        printk("--CWMCU-- %s , 0x01\n", __func__);
        if(CWMCU_I2C_R(sensor, RegMapR_PEELIRLearnStatus, irGetData, 4) < 0) {
            printk("--CWMCU-- write IR send learning start fail. [i2c]\n");
        }

        memset(sensor->ir_ReportData,0,sizeof(uint8_t)*2048);
		
		sensor->ir_data_length = 4;

        //IR learning reply
        if(irGetData[1] == 0) {
           
            memcpy(sensor->ir_ReportData,irGetData,sizeof(uint8_t)*4);        
            printk("--CWMCU-- IR learning idle [0]: %x \n",irGetData[1]);
            
        }else if(irGetData[1] == 1){
		    
			memcpy(sensor->ir_ReportData,irGetData,sizeof(uint8_t)*4);
            
            printk("--CWMCU-- IR learning Done [0]: %x\n", irGetData[1]);
            
		}else if(irGetData[1] == 2){
		  
		    memcpy(sensor->ir_ReportData,irGetData,sizeof(uint8_t)*4);
            
            printk("--CWMCU-- Transmit in Progress [0]: %x\n",irGetData[1]);
		  
		}else if (irGetData[1] == 3){
		   
		    memcpy(sensor->ir_ReportData,irGetData,sizeof(uint8_t)*4);
            
            printk("--CWMCU-- Learning in Progress [0]: %x\n",irGetData[1]);
		
		}
        else {
		    printk("--CWMCU-- IR Get learning info not ready [0]: %x\n",irGetData[1]);
            sensor->ir_ReportData[1] = 255;
        }
        
        irstatus = irGetData[1];
    }
	
    if(sensor->irInputData[0]==0x50 && sensor->irInputData[6]==0x4C && sensor->irInputData[7]==0x02) {
        printk("--CWMCU-- %s , 0x02\n", __func__);
		sensor->ir_data_length = ((uint16_t)sensor->irInputData[2] << 8) | (uint16_t)sensor->irInputData[3]; 
        memset(irPacket,0,sizeof(uint8_t)*128);
        loops = (sensor->ir_data_length / 124) + 1;
        printk("--CWMCU-- %s , loop:%d ir_data_length:%d\n", __func__, loops, sensor->ir_data_length);
		offset=7;
        for(i = 0; i < loops; i++) {
            irPacket[0] = loops;
            irPacket[1] = i;        

            if( (i+1) == loops) {
                irPacket[2] = (sensor->ir_data_length%124); //transfer size
            } else {
                irPacket[2] = 124;
            }

            memcpy(irPacket+3,sensor->irInputData+offset,sizeof(uint8_t)*irPacket[2]);
			offset +=irPacket[2];
			checksum=0;

			for (j = 0; j < 124; j++) {
                checksum += irPacket[j+3];
            }
            irPacket[127] = checksum;

            if(CWMCU_write_i2c_multiple(sensor, RegMapW_PEELSendIRData, irPacket, 128) < 0) {
                printk("--CWMCU-- write IR data fail. [i2c]\n");
            }
            usleep(5000);
			printk("--CWMCU-- irPacket[0-4]:%d %d %d %d %d\n", irPacket[0], irPacket[1], irPacket[2], irPacket[3], irPacket[4]);
        }
		
		if(i == loops)
		{
         memset(irPacket,0,sizeof(uint8_t)*128);
         irPacket[0] = 1;
         if(CWMCU_I2C_W(sensor, RegMapW_PEELSendIRGo, irPacket, 1) < 0) {
            printk("--CWMCU-- write IR data fail. [i2c]\n");
         }
		}
    }

    //
    //for ir learning
    //

    //enable IR learning
    if(sensor->irInputData[0]==0x50 && sensor->irInputData[6]==0x4C && sensor->irInputData[7]==0x03) {
        printk("--CWMCU-- %s , 0x03\n", __func__);
        memset(irPacket,0,sizeof(uint8_t)*128);
        irPacket[0] = 1;
        if(CWMCU_I2C_W(sensor, RegMapW_PEELIRLearnEn, irPacket, 1) < 0) {
            printk("--CWMCU-- write IR send learning start fail. [i2c]\n");
        }
    }
	
	if(sensor->irInputData[0]==0x50 && sensor->irInputData[6]==0x4C && sensor->irInputData[7]==0x04) {
        printk("--CWMCU-- %s , 0x04\n", __func__);
        memset(irPacket,0,sizeof(uint8_t)*128);
        irPacket[0] = 1;
        if(CWMCU_I2C_W(sensor, RegMapW_PEELSendIRSTOP, irPacket, 1) < 0) {
            printk("--CWMCU-- write IR data fail. [i2c]\n");
        }
    }

    if(sensor->irInputData[0]==0x50 && sensor->irInputData[6]==0x4C && sensor->irInputData[7]==0x05) {
        printk("--CWMCU-- %s , 0x05\n", __func__);
        //offset = 0;
		
	 if(CWMCU_I2C_R(sensor, RegMapR_PEELIRLearnStatus, irGetData, 4) < 0) {
           	 printk("--CWMCU-- write IR send learning start fail. [i2c]\n");
       	 }
	

	if(irGetData[1]  == 1) {

	        sensor->ir_data_length = (uint16_t)((irGetData[2] << 8) | irGetData[3]);
	        //memcpy(sensor->ir_ReportData,&sensor->ir_data_length,2);
	        //offset += 2;

	        loops = (sensor->ir_data_length / 124) + 1;	
	        printk("--CWMCU-- 77 %s , loop:%d\n", __func__, loops);
	        //memcpy(sensor->ir_ReportData+offset,sensor->irInputData,3);
	        offset = 0;
	        for (i = 0; i < loops; i++) {

	            if(CWMCU_read_i2c_multiple(sensor, RegMapR_PEELIRLearnGetData, irPacket, 128) < 0) {
	                printk("--CWMCU-- Read IR learning data fail. [i2c]\n");
	            }
				
	            memcpy(sensor->ir_ReportData+offset,irPacket+3,irPacket[2]);
	            offset +=irPacket[2];		    
	            checksum = 0;
	            for (j = 0; j < 124; j++) {
	                checksum += irPacket[j+3];
	                //printk("--CWMCU-- checksum : %d, data[%d]: 0x%x\n", checksum, j+3, irPacket[j+3]);
	            }
	            if(checksum != irPacket[127]) {
	                printk("--CWMCU-- learning data checksum fail.[%d] checksum:%u | %u\n",i,checksum,irPacket[127]);
	            } else {
	                printk("--CWMCU-- learning data checksum pass.[%d] size=%d checksum:%u | %u\n",i,irPacket[2],checksum,irPacket[127]);
	            }
	            usleep(5000);
	        }
	      }
    }

	//Test with Buf0
	if(sensor->irInputData[0]==0x50 && sensor->irInputData[6]==0x4C && sensor->irInputData[7]==0x06) {
		printk("--CWMCU-- %s , 0x02\n", __func__);
		sensor->ir_data_length = sizeof(Buf0);//((uint16_t)sensor->irInputData[2] << 8) | (uint16_t)sensor->irInputData[3]; 
		memset(irPacket,0,sizeof(uint8_t)*128);
		loops = (sensor->ir_data_length / 124) + 1;
		printk("--CWMCU-- %s , loop:%d ir_data_length:%d\n", __func__, loops, sensor->ir_data_length);
		//offset=8;
		offset = 0; 
		for(i = 0; i < loops; i++) {
			irPacket[0] = loops;
			irPacket[1] = i;		

			if( (i+1) == loops) {
				irPacket[2] = (sensor->ir_data_length%124); //transfer size
			} else {
				irPacket[2] = 124;
			}

			memcpy(irPacket+3, Buf0+offset,sizeof(uint8_t)*irPacket[2]);
			offset +=irPacket[2];
			checksum=0;
			//for (j = 0; j < 120; j++) {
			for (j = 0; j < 124; j++) {
				checksum += irPacket[j+3];
			}
			irPacket[127] = checksum;

			if(CWMCU_write_i2c_multiple(sensor, RegMapW_PEELSendIRData, irPacket, 128) < 0) {
				printk("--CWMCU-- write IR data fail. [i2c]\n");
			}
			usleep(5000);
			printk("--CWMCU-- irPacket[0-4]:%x %x %x %x %x \n", irPacket[0], irPacket[1], irPacket[2], irPacket[3], irPacket[4]);
			printk("--CWMCU-- irPacket[5-9]:%x %x %x %x %x \n", irPacket[5], irPacket[6], irPacket[7], irPacket[8], irPacket[9]);
			printk("--CWMCU-- irPacket[10-14]:%x %x %x %x %x \n", irPacket[10], irPacket[11], irPacket[12], irPacket[13], irPacket[14]);
		}
		
		if(i == loops)
		{
			 memset(irPacket,0,sizeof(uint8_t)*128);
			 irPacket[0] = 1;
			 if(CWMCU_I2C_W(sensor, RegMapW_PEELSendIRGo, irPacket, 1) < 0) {
				printk("--CWMCU-- write IR data fail. [i2c]\n");
			 }
		}
	}
	
    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);
	
    return count;
}

static struct device_attribute attributes[] = {
    __ATTR(enable, 0666,  active_show, active_set),
    __ATTR(batch, 0220, NULL, batch_set),
    __ATTR(flush, 0220, NULL, flush_set),
    __ATTR(mcu_mode, 0666, mcu_mode_show, mcu_model_set),
    __ATTR(calibrator_cmd, 0666,  get_calibrator_cmd, set_calibrator_cmd),
    __ATTR(calibrator_data, 0666, get_calibrator_data, set_calibrator_data),
    __ATTR(firmware_update_i2c, 0666, get_firmware_update_i2, set_firmware_update_i2),
    __ATTR(firmware_update_cmd, 0220, NULL, set_firmware_update_cmd),
    __ATTR(firmware_update_data, 0220, NULL, set_firmware_update_data),
    __ATTR(firmware_update_status, 0440, get_firmware_update_status, NULL),
    __ATTR(version, 0440,  version_show, NULL),
    __ATTR(library_version, 0666,  library_version_show, NULL),
    __ATTR(timestamp, 0440, timestamp_show, NULL),
    __ATTR(sys_cmd, 0220,  NULL, set_sys_cmd),
    __ATTR(raw_data0, 0440, get_raw_data0, NULL),
    __ATTR(raw_data1, 0440, get_raw_data1, NULL),
    __ATTR(raw_data2, 0440, get_raw_data2, NULL),
    __ATTR(raw_data3, 0440, get_raw_data3, NULL),
    __ATTR(raw_data4, 0440, get_raw_data4, NULL),
    __ATTR(irc, 0666, irc_get, irc_set),
	__ATTR(irp, 0666, peel_irc_get, peel_irc_set),
    __ATTR(enable_log, 0666, enable_log_get, enable_log_set),
};

static void CWMCU_IRQ(struct CWMCU_T *sensor)
{
    uint8_t temp[2] = {0};
    uint8_t data_event[24] = {0};
    

    if (CWMCU_I2C_R(sensor, RegMapR_InterruptStatus, temp, 2) >= 0)
    {
        sensor->interrupt_status = (u32)temp[1] << 8 | (u32)temp[0];
        if (sensor->debug_log) 
           SH_LOG("interrupt_status:%d\n", sensor->interrupt_status);
    }
    else
    {
        SH_ERR("check interrupt_status failed\n");
        return;
    }
               
    if (sensor->interrupt_status & (1<<IRQ_INIT))
    {
        cwmcu_reinit(sensor);
        cw_send_event(sensor, NonWakeUpHandle, MCU_REINITIAL, data_event);
    }

    if (sensor->interrupt_status & (1<<IRQ_GESTURE))
    {
        cwmcu_read_gesture(sensor);
    }

    if ((sensor->interrupt_status & (1<<IRQ_BATCH_TIMEOUT)) ||(sensor->interrupt_status & (1<<IRQ_BATCH_FULL)) ) 
    {
        cwmcu_read_buff(sensor,WakeUpHandle);
    }

    if (sensor->interrupt_status & (1<<IRQ_INFO))
    {
        read_mcu_info(sensor);
    }
    if (sensor->interrupt_status & (1<<IRQ_CALIB)) 
    {
        read_calib_info(sensor);
    }
}

static int CWMCU_suspend(struct device *dev)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    
    cancel_delayed_work_sync(&sensor->delay_work);
    
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 1);
    cwmcu_kernel_status(sensor,KERNEL_SUPEND);
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 0);
    return 0;
}

static int CWMCU_resume(struct device *dev)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 1);
    cwmcu_kernel_status(sensor,KERNEL_RESUND);
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 0);
    queue_delayed_work(sensor->driver_wq, &sensor->delay_work,
        msecs_to_jiffies(atomic_read(&sensor->delay)));
    return 0;
}

/*=======iio device reg=========*/
static void iio_trigger_work(struct irq_work *work)
{
    struct CWMCU_T *mcu_data = container_of((struct irq_work *)work, struct CWMCU_T, iio_irq_work);

    iio_trigger_poll(mcu_data->trig, iio_get_time_ns());
}

static irqreturn_t cw_trigger_handler(int irq, void *p)
{
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

#ifdef CWMCU_MUTEX
    mutex_lock(&mcu_data->mutex_lock);
#endif
    iio_trigger_notify_done(mcu_data->indio_dev->trig);
#ifdef CWMCU_MUTEX
    mutex_unlock(&mcu_data->mutex_lock);
#endif

    return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops cw_buffer_setup_ops = {
    .preenable = &iio_sw_buffer_preenable,
    .postenable = &iio_triggered_buffer_postenable,
    .predisable = &iio_triggered_buffer_predisable,
};

static int cw_pseudo_irq_enable(struct iio_dev *indio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

    if (!atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 0, 1))
    {
        SH_FUN();
		cancel_work_sync(&mcu_data->work);
		queue_work(mcu_data->driver_wq, &mcu_data->work);
        cancel_delayed_work_sync(&mcu_data->delay_work);
        queue_delayed_work(mcu_data->driver_wq, &mcu_data->delay_work, 0);
    }

    return 0;
}

static int cw_pseudo_irq_disable(struct iio_dev *indio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

    if (atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 1, 0))
    {
		cancel_work_sync(&mcu_data->work);
        cancel_delayed_work_sync(&mcu_data->delay_work);
        SH_FUN();
    }
    return 0;
}

static int cw_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
    if (enable)
        cw_pseudo_irq_enable(indio_dev);
    else
        cw_pseudo_irq_disable(indio_dev);
    return 0;
}

static int cw_data_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
    struct iio_dev *indio_dev = (struct iio_dev *)iio_trigger_get_drvdata(trig);
#ifdef CWMCU_MUTEX
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);
    mutex_lock(&mcu_data->mutex_lock);
#endif
    cw_set_pseudo_irq(indio_dev, state);
#ifdef CWMCU_MUTEX
    mutex_unlock(&mcu_data->mutex_lock);
#endif
    
    return 0;
}

static const struct iio_trigger_ops cw_trigger_ops = {
    .owner = THIS_MODULE,
    .set_trigger_state = &cw_data_rdy_trigger_set_state,
};

static int cw_probe_trigger(struct iio_dev *iio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(iio_dev);
    int ret;

    iio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time, &cw_trigger_handler, 
                            IRQF_ONESHOT, iio_dev, "%s_consumer%d", iio_dev->name, iio_dev->id);
    if (NULL == iio_dev->pollfunc)
    {
        ret = -ENOMEM;
        goto error_ret;
    }

    mcu_data->trig = iio_trigger_alloc("%s-dev%d",
            iio_dev->name,
            iio_dev->id);
    if (!mcu_data->trig) {
        ret = -ENOMEM;
        goto error_dealloc_pollfunc;
    }
    mcu_data->trig->dev.parent = &mcu_data->client->dev;
    mcu_data->trig->ops = &cw_trigger_ops;
    iio_trigger_set_drvdata(mcu_data->trig, iio_dev);

    ret = iio_trigger_register(mcu_data->trig);
    if (ret)
        goto error_free_trig;

    return 0;

error_free_trig:
    iio_trigger_free(mcu_data->trig);
error_dealloc_pollfunc:
    iio_dealloc_pollfunc(iio_dev->pollfunc);
error_ret:
    return ret;
}

static int cw_probe_buffer(struct iio_dev *iio_dev)
{
    int ret;
    struct iio_buffer *buffer;

    buffer = iio_kfifo_allocate(iio_dev);
    if (!buffer)
    {
        ret = -ENOMEM;
        goto error_ret;
    }

    buffer->scan_timestamp = true;
    iio_dev->buffer = buffer;
    iio_dev->setup_ops = &cw_buffer_setup_ops;
    iio_dev->modes |= INDIO_BUFFER_TRIGGERED;
    ret = iio_buffer_register(iio_dev, iio_dev->channels,
                  iio_dev->num_channels);
    if (ret)
        goto error_free_buf;

    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_ID);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_X);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Y);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Z);
    return 0;

error_free_buf:
    iio_kfifo_free(iio_dev->buffer);
error_ret:
    return ret;
}

static int cw_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                            int *val, int *val2, long mask)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);
    int ret = -EINVAL;

    if (chan->type != IIO_ACCEL)
        return ret;

#ifdef CWMCU_MUTEX
    mutex_lock(&mcu_data->mutex_lock);
#endif
    switch (mask)
    {
    case 0:
        *val = mcu_data->iio_data[chan->channel2 - IIO_MOD_X];
        ret = IIO_VAL_INT;
        break;

    case IIO_CHAN_INFO_SCALE:
        /* Gain : counts / uT = 1000 [nT] */
        /* Scaling factor : 1000000 / Gain = 1000 */
        *val = 0;
        *val2 = 1000;
        ret = IIO_VAL_INT_PLUS_MICRO;
        break;

        default:
            break;
    }
#ifdef CWMCU_MUTEX
    mutex_unlock(&mcu_data->mutex_lock);
#endif
    return ret;
}

#define CW_CHANNEL(axis)                    \
{                                           \
    .type = IIO_ACCEL,                      \
    .modified = 1,                          \
    .channel2 = axis+1,                     \
    .info_mask = BIT(IIO_CHAN_INFO_SCALE),  \
    .scan_index = axis,                     \
    .scan_type = IIO_ST('u', 32, 32, 0)     \
}

static const struct iio_chan_spec cw_channels[] = {
    CW_CHANNEL(CW_SCAN_ID),
    CW_CHANNEL(CW_SCAN_X),
    CW_CHANNEL(CW_SCAN_Y),
    CW_CHANNEL(CW_SCAN_Z),
    IIO_CHAN_SOFT_TIMESTAMP(CW_SCAN_TIMESTAMP)
};

static const struct iio_info cw_info = {
    .read_raw = &cw_read_raw,
    .driver_module = THIS_MODULE,
};

static void cwmcu_delwork_report(struct work_struct *work)
{
    struct CWMCU_T *sensor = container_of((struct delayed_work *)work,
        struct CWMCU_T, delay_work);
    
    if (atomic_read(&sensor->pseudo_irq_enable)) {
        if (sensor->mcu_mode == CW_BOOT) {
            printk("%s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);
        }else{
            power_pin_sw(sensor,SWITCH_POWER_POLLING, 1);
            CWMCU_POLLING(sensor);
            power_pin_sw(sensor,SWITCH_POWER_POLLING, 0);

        }
    queue_delayed_work(sensor->driver_wq, &sensor->delay_work,
    msecs_to_jiffies(atomic_read(&sensor->delay)));
    }
}

static int create_sysfs_interfaces(struct CWMCU_T *mcu_data)
{
    int i;
    int res;

    SH_FUN();
    mcu_data->sensor_class = class_create(THIS_MODULE, "cywee_sensorhub");
    if (IS_ERR(mcu_data->sensor_class))
        return PTR_ERR(mcu_data->sensor_class);

    mcu_data->sensor_dev = device_create(mcu_data->sensor_class, NULL, 0, "%s", "sensor_hub");
    if (IS_ERR(mcu_data->sensor_dev))
    {
        res = PTR_ERR(mcu_data->sensor_dev);
        goto err_device_create;
    }

    res = dev_set_drvdata(mcu_data->sensor_dev, mcu_data);
    if (res)
        goto err_set_drvdata;

    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        if (device_create_file(mcu_data->sensor_dev, attributes + i))
            goto error;

    res = sysfs_create_link(&mcu_data->sensor_dev->kobj, &mcu_data->indio_dev->dev.kobj, "iio");
    if (res < 0)
        goto error;

    return 0;

error:
    while (--i >= 0)
        device_remove_file(mcu_data->sensor_dev, attributes + i);
err_set_drvdata:
    put_device(mcu_data->sensor_dev);
    device_unregister(mcu_data->sensor_dev);
err_device_create:
    class_destroy(mcu_data->sensor_class);
    return res;
}

#ifdef CWMCU_INTERRUPT
static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
    struct CWMCU_T *sensor = data;
    // printk(KERN_DEBUG "CwMcu:%s in\n", __func__); 
    if (sensor->mcu_mode == CW_BOOT) {
        printk("%s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);
        return IRQ_HANDLED;
    }
    schedule_work(&sensor->work);

    return IRQ_HANDLED;
}

static void cwmcu_work_report(struct work_struct *work)
{
  
   struct CWMCU_T *sensor = container_of((struct work_struct *)work,
            struct CWMCU_T, work);
		
    if (sensor->mcu_mode == CW_BOOT) {
        printk("%s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);
        return;
    }
    power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 1);        
    CWMCU_IRQ(sensor);
    power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 0);

}
#endif

static int cwstm_parse_dt(struct device *dev,
             struct CWMCU_T *sensor)
{
    struct device_node *np = dev->of_node;
    int ret = 0;

    ret = of_get_named_gpio(np, "cwstm,irq-gpio", 0);
    if (ret < 0) {
        pr_err("failed to get \"cwstm,irq_gpio\"\n");
        goto err;
    }
    sensor->irq_gpio = ret;
    ret = of_get_named_gpio(np, "cwstm,wakeup-gpio", 0);
    if (ret < 0) {
        pr_err("failed to get \"wakeup\"\n");
        goto err;
    }
    sensor->wakeup_gpio = ret;

/*************zhujp2 add start*************************/
        ret = of_get_named_gpio(np, "cwstm,boot-gpio", 0);
        if (ret < 0) {
                pr_err("failed to get \"reset\"\n");
                goto err;
        }
        sensor->boot_gpio = ret;

        ret = of_get_named_gpio(np, "cwstm,reset-gpio", 0);
        if (ret < 0) {
                pr_err("failed to get \"reset\"\n");
                goto err;
        }
        sensor->reset_gpio = ret;
/*************zhujp2 add end**************************/
err:
    return ret;
}

static void cwmcu_remove_trigger(struct iio_dev *indio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

    iio_trigger_unregister(mcu_data->trig);
    iio_trigger_free(mcu_data->trig);
    iio_dealloc_pollfunc(indio_dev->pollfunc);
}

static void cwmcu_remove_buffer(struct iio_dev *indio_dev)
{
    iio_buffer_unregister(indio_dev);
    iio_kfifo_free(indio_dev->buffer);
}


static int cwstm_power_on(struct CWMCU_T *sensor,bool on)
{
    int rc;

    if (!on)
        goto power_off;

    rc = regulator_enable(sensor->vdd);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }

 /************zhujp2 add*************/
    rc = regulator_enable(sensor->vdd_sensors);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }

/*
    rc = regulator_enable(sensor->vdd_ir);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }
*/
/**************zhujp2 add end**********/

    rc = regulator_enable(sensor->vcc_i2c);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vcc_i2c enable failed rc=%d\n", rc);
        regulator_disable(sensor->vdd);
    }

    return rc;


power_off:
    rc = regulator_disable(sensor->vdd);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }
/***********zhujp2 add ****************/
    rc = regulator_disable(sensor->vdd_sensors);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }
/*
    rc = regulator_disable(sensor->vdd_ir);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }
*/
/*************zhujp2 add end***************/
    rc = regulator_disable(sensor->vcc_i2c);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vcc_i2c disable failed rc=%d\n", rc);
        regulator_disable(sensor->vdd);
    }

    return rc;
}

static int cwstm_power_init(struct CWMCU_T *sensor,bool on)
{
    int rc;

    if (!on)
        goto pwr_deinit;

    sensor->vdd = regulator_get(&sensor->client->dev, "vdd");
    if (IS_ERR(sensor->vdd)) {
        rc = PTR_ERR(sensor->vdd);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd) > 0) {
        rc = regulator_set_voltage(sensor->vdd, FT_VTG_MIN_UV,
                       FT_VTG_MAX_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
                "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_put;
        }
    }

    sensor->vcc_i2c = regulator_get(&sensor->client->dev, "vcc_i2c");
    if (IS_ERR(sensor->vcc_i2c)) {
        rc = PTR_ERR(sensor->vcc_i2c);
        dev_err(&sensor->client->dev,
            "Regulator get failed vcc_i2c rc=%d\n", rc);
        goto reg_vdd_set_vtg;
    }

    if (regulator_count_voltages(sensor->vcc_i2c) > 0) {
        rc = regulator_set_voltage(sensor->vcc_i2c, FT_I2C_VTG_MIN_UV,
                       FT_I2C_VTG_MAX_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
            "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
            goto reg_vcc_i2c_put;
        }
    }

 /**************zhujp2 add*****************/

    sensor->vdd_sensors = regulator_get(&sensor->client->dev, "vdd_sensors");
    if (IS_ERR(sensor->vdd_sensors)) {
        rc = PTR_ERR(sensor->vdd_sensors);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd_sensors) > 0) {
        rc = regulator_set_voltage(sensor->vdd_sensors, FT_VTG_MIN_B_UV,
                       FT_VTG_MAX_B_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
                "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_sensors_put;
        }
    }

/*
    sensor->vdd_ir = regulator_get(&sensor->client->dev, "vdd_ir");
    if (IS_ERR(sensor->vdd_ir)) {
        rc = PTR_ERR(sensor->vdd_ir);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd_ir) > 0) {
        rc = regulator_set_voltage(sensor->vdd_ir, FT_VTG_MIN_C_UV,
                       FT_VTG_MAX_C_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
                "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_ir_put;
        }
    }
*/


/****************zhujp2 add end*************/
    return 0;

reg_vcc_i2c_put:
    regulator_put(sensor->vcc_i2c);
reg_vdd_set_vtg:
    if (regulator_count_voltages(sensor->vdd) > 0)
        regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
    regulator_put(sensor->vdd);
reg_vdd_sensors_put:                                //zhujp2 add
    regulator_put(sensor->vdd_sensors);
/*
reg_vdd_ir_put:                                //zhujp2 add
    regulator_put(sensor->vdd_ir);
*/
    return rc;

pwr_deinit:
    if (regulator_count_voltages(sensor->vdd) > 0)
        regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);

    regulator_put(sensor->vdd);

    if (regulator_count_voltages(sensor->vcc_i2c) > 0)
        regulator_set_voltage(sensor->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

    regulator_put(sensor->vcc_i2c);
    return 0;
}

static void cwmcu_hw_config_init(struct CWMCU_T *sensor)
{
    int i = 0;
    int j = 0;
    
    sensor->initial_hw_config = 0;
    for(i = 0; i < HANDLE_ID_END; i++)
    {
        sensor->enabled_list[i] = 0;
        for(j = 0;j<SENSORS_ID_END;j++)
        {
            sensor->sensors_info[i][j].en = 0;
            sensor->sensors_info[i][j].mode= 0;
            sensor->sensors_info[i][j].rate = 0;
            sensor->sensors_info[i][j].timeout= 0;
        }
    }

    sensor->interrupt_status = 0;
    sensor->power_on_list = 0;
    sensor->cal_cmd = 0;
    sensor->cal_type = 0;
    sensor->cal_id = 0;
//    sensor->debug_log = log_flag;
    for(i = 0;i<DRIVER_ID_END;i++){
        sensor->hw_info[i].hw_id=0;
        
        sensor->calibratorUpdate[i]=0;
        for(j = 0;j<30;j++)
        {
            sensor->calibratordata[i][j]=0;
        }
    }

}

static int CWMCU_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    
    struct CWMCU_T *mcu;
    struct iio_dev *indio_dev;
    int error;
    
    printk("%s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);

    dev_dbg(&client->dev, "%s:\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "CwMcu: i2c_check_functionality error\n");
        return -EIO;
    }

    indio_dev = iio_device_alloc(sizeof(*mcu));
    if (!indio_dev) {
        printk("%s: iio_device_alloc failed\n", __func__);
        return -ENOMEM;
    }

    i2c_set_clientdata(client, indio_dev);

    indio_dev->name = CWMCU_I2C_NAME;
    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &cw_info;
    indio_dev->channels = cw_channels;
    indio_dev->num_channels = ARRAY_SIZE(cw_channels);
    indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
    
    mcu = iio_priv(indio_dev);
    mcu->client = client;
    mcu->indio_dev = indio_dev;

/**************************zhujp2 modify********************/
    error = cwstm_power_init(mcu,true);

    if (error) {
        dev_err(&client->dev, "power init failed");
    }
    error = cwstm_power_on(mcu,true);
    if (error) {
        dev_err(&client->dev, "power on failed");
    }
/*****************************zhujp2 modify*****************/
    error = cwstm_parse_dt(&client->dev, mcu);
    if (error < 0) {
        pr_err("failed to parse device tree: %d\n", error);
        goto err_parse_dt;
    }
    gpio_request(mcu->wakeup_gpio, "cwstm,wakeup-gpio");
    printk("wakeup_gpio gpio_request success !!!!!!!!!!!!!! \n");
    gpio_request(mcu->irq_gpio, "cwstm,irq-gpio");
/************zhujp2 add ********/
    gpio_request(mcu->boot_gpio, "cwstm,boot-gpio");
    gpio_direction_output(mcu->boot_gpio, 0);
    gpio_set_value(mcu->boot_gpio, 0);
    printk("boot-gpio  gpio_request success !!!!!!!!!!!!!! \n");
    gpio_request(mcu->reset_gpio, "cwstm,reset-gpio");
    gpio_direction_output(mcu->reset_gpio, 1);
    gpio_set_value(mcu->reset_gpio, 1);
	msleep(2);
	gpio_set_value(mcu->reset_gpio, 0);
	msleep(2);
	gpio_set_value(mcu->reset_gpio, 1);
	msleep(2);
   printk("reset-gpio  gpio_request success !!!!!!!!!!!!!! \n");
/************zhujp2 add end ********/

/***************************zhujp2 modify 
    error = cwstm_power_init(mcu,true);  
    if (error) {
        dev_err(&client->dev, "power init failed");
    }

    error = cwstm_power_on(mcu,true);
    if (error) {
        dev_err(&client->dev, "power on failed");
    }
**************************************/
    atomic_set(&mcu->delay, 20);
    INIT_DELAYED_WORK(&mcu->delay_work, cwmcu_delwork_report);

#ifdef CWMCU_MUTEX
    mutex_init(&mcu->mutex_lock);
    mutex_init(&mcu->mutex_lock_i2c);
#endif
    mcu->supend_flag = 0;
    mcu->mcu_mode = CW_NORMAL;

    error = cw_probe_buffer(indio_dev);
    if (error) {
        printk("%s: iio yas_probe_buffer failed\n", __func__);
        goto error_free_dev;
    }
    error = cw_probe_trigger(indio_dev);
    if (error) {
        printk("%s: iio yas_probe_trigger failed\n", __func__);
        goto error_remove_buffer;
    }
    error = iio_device_register(indio_dev);
    if (error) {
        printk("%s: iio iio_device_register failed\n", __func__);
        goto error_remove_trigger;
    }

    error = create_sysfs_interfaces(mcu);
    if (error)
        goto err_free_mem;

    init_irq_work(&mcu->iio_irq_work, iio_trigger_work);

    cwmcu_hw_config_init(mcu);
    
    mcu->driver_wq = create_singlethread_workqueue("cywee_mcu");
    i2c_set_clientdata(client, mcu);
    pm_runtime_enable(&client->dev);

#ifdef CWMCU_INTERRUPT
    mcu->client->irq =gpio_to_irq(mcu->irq_gpio);

    gpio_direction_output(mcu->irq_gpio, 1);
    usleep(20000);
    pr_info("%s:irq gpio vlaue %d\n", __func__, gpio_get_value(mcu->irq_gpio));
    gpio_direction_input(mcu->irq_gpio);
    usleep(5000);

    printk("%s:%s:(sensor->client->irq  =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__, mcu->client->irq);


        error = request_threaded_irq(mcu->client->irq, NULL,
                           CWMCU_interrupt_thread,
                           IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                           "cwmcu", mcu);
        if (error < 0) {
                pr_err("request irq %d failed\n", mcu->client->irq);
               // goto exit_destroy_mutex;
        }

        INIT_WORK(&mcu->work, cwmcu_work_report);
        
        error = enable_irq_wake(mcu->client->irq);
        if (error < 0) 
            printk("[CWMCU] could not enable irq as wakeup source %d\n", error);

#endif

    printk("%s:%s:(probe success)\n",LOG_TAG_KERNEL ,__FUNCTION__);

    return 0;

err_free_mem:
    iio_device_unregister(indio_dev);
error_remove_trigger:
    cwmcu_remove_trigger(indio_dev);
error_remove_buffer:
    cwmcu_remove_buffer(indio_dev);
error_free_dev:
err_parse_dt:
#ifdef CWMCU_INTERRUPT
//exit_destroy_mutex:
#endif
    iio_device_free(indio_dev);
    i2c_set_clientdata(client, NULL);
    printk("--CWMCU-- sensorhub probe fail.\n");
    return error;
}

static int CWMCU_i2c_remove(struct i2c_client *client)
{
    struct CWMCU_T *sensor = i2c_get_clientdata(client);
    kfree(sensor);
    return 0;
}

static struct of_device_id cwstm_match_table[] = {
    { .compatible = "cwstm,cwstm32",},
    { },
};

static const struct dev_pm_ops CWMCU_pm_ops = {
    .suspend = CWMCU_suspend,
    .resume = CWMCU_resume
};

static const struct i2c_device_id CWMCU_id[] = {
    { CWMCU_I2C_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, CWMCU_id);

static struct i2c_driver CWMCU_driver = {
    .driver = {
        .name = CWMCU_I2C_NAME,
        .owner = THIS_MODULE,
        .pm = &CWMCU_pm_ops,
        .of_match_table = cwstm_match_table,
    },
    .probe    = CWMCU_i2c_probe,
    .remove   = CWMCU_i2c_remove,
    .id_table = CWMCU_id,
};

static int __init CWMCU_i2c_init(void){
   // printk("%s:%s:(init)\n",LOG_TAG_KERNEL ,__FUNCTION__);
    return i2c_add_driver(&CWMCU_driver);
}

static void __exit CWMCU_i2c_exit(void){
    i2c_del_driver(&CWMCU_driver);
}

module_init(CWMCU_i2c_init);
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver");
MODULE_AUTHOR("CyWee Group Ltd.");

