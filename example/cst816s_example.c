
#include <rtthread.h>
#include <rtdevice.h>
#include "cst816s.h"

#define THREAD_PRIORITY 25
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 5

static rt_thread_t cst816s_thread = RT_NULL;
static rt_sem_t cst816s_sem = RT_NULL;
static rt_touch_t touch_device = RT_NULL;
static struct rt_touch_data read_data;
static struct rt_touch_info info;

#include "lvgl.h"
extern lv_obj_t *label0;

static void cst816s_entry(void *parameter)
{
    while (1)
    {
        rt_sem_take(cst816s_sem, RT_WAITING_FOREVER);
        touch_device->ops->touch_readpoint(touch_device, &read_data, RT_NULL);
        rt_uint32_t tick_old;
        if (read_data.timestamp != tick_old)
        {
            lv_obj_set_pos(label0, read_data.x_coordinate, read_data.y_coordinate);
            rt_kprintf("%d %3d %3d %d \r\n", read_data.track_id, read_data.x_coordinate, read_data.y_coordinate, read_data.timestamp);

            tick_old = read_data.timestamp;
        }
        rt_device_control(touch_device, RT_TOUCH_CTRL_ENABLE_INT, RT_NULL);
    }
}

static rt_err_t rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(cst816s_sem);
    rt_device_control(dev, RT_TOUCH_CTRL_DISABLE_INT, RT_NULL);
    return 0;
}

/* Test function */
static void cst816s_sample(void)
{
    touch_device = rt_device_find(TOUCH_DEVICE_NAME);
    if (touch_device == RT_NULL)
    {
        rt_kprintf("can't find device:%s\n", TOUCH_DEVICE_NAME);
        return -1;
    }

    if (rt_device_open(touch_device, RT_DEVICE_FLAG_INT_RX) != RT_EOK) // check device only, touch device has no init function
    {
        rt_kprintf("open device failed!");
        return -1;
    }

    if (rt_device_control(touch_device, RT_TOUCH_CTRL_GET_INFO, &info) == RT_ERROR)
    {
        return -1;
    }

    cst816s_sem = rt_sem_create("dsem", 0, RT_IPC_FLAG_FIFO);
    if (cst816s_sem == RT_NULL)
    {
        rt_kprintf("create dynamic semaphore failed.\n");
        return -1;
    }

    rt_device_set_rx_indicate(touch_device, rx_callback);

    cst816s_thread = rt_thread_create("touch", cst816s_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);

    if (cst816s_thread != RT_NULL)
        rt_thread_startup(cst816s_thread);

    return 0;
}
INIT_DEVICE_EXPORT(cst816s_sample);
