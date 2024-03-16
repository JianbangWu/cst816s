#include <rtthread.h>
#include <rtdevice.h>
#include "cst816s.h"

#define DBG_TAG "cst816s"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct rt_i2c_client *cst816s_client;

static rt_err_t cst816s_write_reg(struct rt_i2c_client *dev, rt_uint8_t *write_data, rt_uint8_t write_len)
{
    struct rt_i2c_msg msgs;

    msgs.addr = dev->client_addr;
    msgs.flags = RT_I2C_WR;
    msgs.buf = write_data;
    msgs.len = write_len;

    if (rt_i2c_transfer(dev->bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}

static rt_err_t cst816s_read_regs(struct rt_i2c_client *dev, rt_uint8_t *cmd_buf, rt_uint8_t cmd_len, rt_uint8_t *read_buf, rt_uint8_t read_len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = dev->client_addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = cmd_buf;
    msgs[0].len = cmd_len;

    msgs[1].addr = dev->client_addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = read_buf;
    msgs[1].len = read_len;

    if (rt_i2c_transfer(dev->bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }
}


static rt_size_t cst816s_read_point(struct rt_touch_device *touch, void *buf, rt_size_t touch_num)
{
    rt_uint8_t point_status = 0;
    struct rt_touch_data *pdata = buf;
    rt_uint8_t read[4];
    rt_uint8_t cmd[2];

    cmd[0] = CST816S_FingerNum;

    cst816s_read_regs(cst816s_client, cmd, 1, read, 1);

    if (read[0] == 0)
    {
        LOG_D("no touch point\n");
    }
    else if (read[0] == 1)
    {
        cmd[0] = CST816S_XposH;
        cst816s_read_regs(cst816s_client, cmd, 1, read, 4);
        pdata->x_coordinate = ((read[0] & 0x0f) << 8) + read[1];
        pdata->y_coordinate = ((read[2] & 0x0f) << 8) + read[3];
        pdata->timestamp =rt_touch_get_ts();
        LOG_D("get touch point\n");
    }
}

static rt_err_t cst816s_init(void)
{
    uint8_t reg_val[5][2] = {0};
    reg_val[0][0] = CST816S_NorScanPer;
    reg_val[0][1] = 0x02;
    reg_val[1][0] = CST816S_IrqCtl;
    reg_val[1][1] = 0x60;
    reg_val[2][0] = CST816S_LongPressTime;
    reg_val[2][1] = 0;
    reg_val[3][0] = CST816S_IOCtl;
    reg_val[3][1] = 0;
    reg_val[4][0] = CST816S_DisAutoSleep;
    reg_val[4][1] = 1;
    cst816s_write_reg(cst816s_client, reg_val, 10);
}

static rt_err_t cst816s_get_info(struct rt_i2c_client *dev, struct rt_touch_info *info)
{
    uint8_t reg = CST816S_ChipID;
    rt_uint8_t reg_val[3] = {0};

    rt_pin_mode(rt_pin_get("PE.1"), PIN_MODE_OUTPUT);
    rt_pin_write(rt_pin_get("PE.1"), PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(rt_pin_get("PE.1"), PIN_HIGH);
    rt_thread_mdelay(30);

    if (cst816s_read_regs(cst816s_client, &reg, 1, reg_val, 3) != RT_EOK)
    {
        LOG_D("read id failed \n");
        return RT_ERROR;
    }
    rt_kprintf("CST816S_ChipID:=%d\r\n", reg_val[0]);
    rt_kprintf("CST816S_ProjID:=%d\r\n", reg_val[1]);
    rt_kprintf("CST816S_FwVersion:=%d\r\n", reg_val[2]);
    cst816s_init();
    return RT_EOK;
}

static rt_err_t cst816s_control(struct rt_touch_device *device, int cmd, void *data)
{

    if (cmd == RT_TOUCH_CTRL_GET_INFO)
    {
        return cst816s_get_info(cst816s_client, data);
    }

    return RT_EOK;
}

static struct rt_touch_ops touch_ops =
    {
        .touch_readpoint = cst816s_read_point,
        .touch_control = cst816s_control,
};

struct rt_touch_config *cfg;

int rt_hw_cst816s_init(const char *name, struct rt_touch_config *cfg)
{
    rt_touch_t touch_device = RT_NULL;

    touch_device = (rt_touch_t)rt_calloc(1, sizeof(struct rt_touch_device));

    if (touch_device == RT_NULL)
        return -RT_ERROR;

    /* hardware init */
    rt_pin_mode(cfg->irq_pin.pin, PIN_MODE_INPUT);
    rt_thread_mdelay(100);

    /* interface bus */
    cst816s_client = (struct rt_i2c_client *)rt_calloc(1, sizeof(struct rt_i2c_client));

    cst816s_client->bus = (struct rt_i2c_bus_device *)rt_device_find(cfg->dev_name);

    if (cst816s_client->bus == RT_NULL)
    {
        LOG_E("Can't find device\r\n");
        return -RT_ERROR;
    }
    else
    {
        LOG_D("Find %s\r\n", cfg->dev_name);
    }
    if (rt_device_open((rt_device_t)cst816s_client->bus, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        LOG_E("open device failed\r\n");
        return -RT_ERROR;
    }
    else
    {
        LOG_D("open device OK\r\n");
    }

    rt_pin_mode(cfg->irq_pin.pin, cfg->irq_pin.mode);

    cst816s_client->client_addr = CST816S_DEV_ADDR;

    /* register touch device */
    touch_device->info.type = RT_TOUCH_TYPE_CAPACITANCE;
    touch_device->info.vendor = RT_TOUCH_VENDOR_UNKNOWN;

    rt_memcpy(&touch_device->config, cfg, sizeof(struct rt_touch_config));
    touch_device->ops = &touch_ops;

    rt_hw_touch_register(touch_device, name, RT_DEVICE_FLAG_INT_RX, RT_NULL);

    LOG_I("touch device cst816s init success\n");

    return RT_EOK;
}

int rt_hw_cst816s_port(void)
{
    struct rt_touch_config config;

    config.dev_name = TOUCH_I2C_NAME;
    config.irq_pin.pin = TOUCH_IRQ_PIN;
    config.irq_pin.mode = PIN_MODE_INPUT;

    config.user_data = RT_NULL;

    rt_hw_cst816s_init(TOUCH_DEVICE_NAME, &config);

    return 0;
}
INIT_PREV_EXPORT(rt_hw_cst816s_port);
