/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-20     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "mlx90392.h"

#include <string.h>
#include <stdlib.h>

/**
 * Lookup table to convert raw values to uT based on [HALLCONF][GAIN_SEL][RES][SENSxy, SENSz].
 */
const float mlx90392_lsb_lookup[2][8][4][2] =
{
    /* HALLCONF = 0xC (default) */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    },

    /* HALLCONF = 0x0 */
    {
        /* GAIN_SEL = 0, 5x gain */
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.392, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    }
};

/**
 * Lookup table for conversion time based on [DIF_FILT][OSR].
 */
const float mlx90392_tconv[8][4] =
{
    /* DIG_FILT = 0 */
    {1.27, 1.84, 3.00, 5.30},
    /* DIG_FILT = 1 */
    {1.46, 2.23, 3.76, 6.84},
    /* DIG_FILT = 2 */
    {1.84, 3.00, 5.30, 9.91},
    /* DIG_FILT = 3 */
    {2.61, 4.53, 8.37, 16.05},
    /* DIG_FILT = 4 */
    {4.15, 7.60, 14.52, 28.34},
    /* DIG_FILT = 5 */
    {7.22, 13.75, 26.80, 52.92},
    /* DIG_FILT = 6 */
    {13.36, 26.04, 51.38, 102.07},
    /* DIF_FILT = 7 */
    {25.65, 50.61, 100.53, 200.37},
};

rt_err_t mlx90392_transfer(struct mlx90392_device *dev, rt_uint8_t *send_buf, rt_uint8_t send_len, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = send_buf;         /* Write data pointer */
        msgs[0].len   = send_len;         /* Number of bytes write */

        msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = recv_buf;         /* Read data pointer */
        msgs[1].len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
        
    }

    return res;
}

rt_err_t mlx90392_reset(struct mlx90392_device *dev)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = CMD_RESET;

    return(mlx90392_transfer(dev, send_buf, 1, recv_buf, 1));
}

rt_err_t mlx90392_start_burst(struct mlx90392_device *dev, rt_int8_t zyxt)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = (CMD_START_BURST)|(zyxt);

    return(mlx90392_transfer(dev, send_buf, 1, recv_buf, 1));
}

rt_err_t mlx90392_wake_on_change(struct mlx90392_device *dev, rt_int8_t zyxt)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = (CMD_WAKE_ON_CHANGE)|(zyxt);

    return(mlx90392_transfer(dev, send_buf, 1, recv_buf, 1));
}

rt_err_t mlx90392_start_measurement(struct mlx90392_device *dev, rt_int8_t zyxt)
{
    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[2];

    send_buf[0] = (CMD_START_MEASUREMENT)|(zyxt);

    return(mlx90392_transfer(dev, send_buf, 1, recv_buf, 1));
}

/**
 * This function reads the value of register for mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90392
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx90392_read_reg(struct mlx90392_device *dev, rt_uint8_t reg, rt_uint16_t *val)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t send_buf[10];
    rt_uint8_t recv_buf[3];

    send_buf[0] = reg;

    res = mlx90392_transfer(dev, send_buf, 1, recv_buf, 1);
    if (res == RT_EOK)
    {
        *val = recv_buf[0];
    }

    return res;
}

static rt_err_t mlx90392_mem_direct_read(struct mlx90392_device *dev, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_RD;        /* Read flag */
        msgs.buf   = recv_buf;         /* Read data pointer */
        msgs.len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }
    else
    {

    }

    return res;
}

static rt_err_t mlx90392_mem_read(struct mlx90392_device *dev, rt_uint8_t start_addr, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf = start_addr;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &send_buf;         /* Write data pointer */
        msgs[0].len   = 1;         /* Number of bytes write */

        msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = recv_buf;         /* Read data pointer */
        msgs[1].len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }
    else
    {

    }

    return res;
}

//send_buf = start register address + data1 + data2 + ...
static rt_err_t mlx90392_mem_write(struct mlx90392_device *dev, rt_uint8_t *send_buf, rt_uint8_t send_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_WR;        /* Read flag */
        msgs.buf   = send_buf;         /* Read data pointer */
        msgs.len   = send_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }
    else
    {

    }

    return res;
}

static rt_err_t mlx90392_address_reset(struct mlx90392_device *dev)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x11;
    send_buf[1] = 0x06;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_WR;        /* Read flag */
        msgs.buf   = send_buf;         /* Read data pointer */
        msgs.len   = 2;                /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            rt_kprintf("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }
    else
    {

    }

    return res;
}

static rt_err_t mlx90392_get_stat1(struct mlx90392_device *dev, union mlx90392_stat1 *stat1)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0x0, (rt_uint8_t *)stat1, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("error\r\n");
    }
    else
    {
        rt_kprintf("STAT1 = 0x%x\r\n", stat1->byte_val);
    }

    return res;
}

static rt_err_t mlx90392_get_stat2(struct mlx90392_device *dev, union mlx90392_stat2 *stat2)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0x7, (rt_uint8_t *)stat2, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("error\r\n");
    }
    else
    {
        rt_kprintf("STAT2 = 0x%x\r\n", stat2->byte_val);
    }

    return res;
}

rt_err_t mlx90392_get_x(struct mlx90392_device *dev, rt_int16_t *x)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x1, recv_buf, 2);
    if (res == RT_EOK)
    {
        *x = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx90392_get_y(struct mlx90392_device *dev, rt_int16_t *y)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x3, recv_buf, 2);
    if (res == RT_EOK)
    {
        *y = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx90392_get_z(struct mlx90392_device *dev, rt_int16_t *z)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x5, recv_buf, 2);
    if (res == RT_EOK)
    {
        *z = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx90392_get_t(struct mlx90392_device *dev, rt_int16_t *t)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x8, recv_buf, 2);
    if (res == RT_EOK)
    {
        *t = recv_buf[1]<<8 | recv_buf[0];
    }

    return res;
}

rt_err_t mlx90392_get_cid(struct mlx90392_device *dev, rt_uint8_t *cid)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0xA, cid, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Read CID is error\r\n");
    }

    return res;
}

rt_err_t mlx90392_get_did(struct mlx90392_device *dev, rt_uint8_t *did)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0xB, did, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Read DID is error\r\n");
    }

    return res;
}

rt_err_t mlx90392_get_mode(struct mlx90392_device *dev, rt_uint8_t *mode)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0x10, mode, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Read MODE is error\r\n");
    }

    return res;
}

rt_err_t mlx90392_set_mode(struct mlx90392_device *dev, enum mlx90392_mode application_mode)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x10;
    send_buf[1] = application_mode;
    res = mlx90392_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("set application mode error\r\n");
    }

    return res;
}

rt_err_t mlx90392_get_osr_dig_filt(struct mlx90392_device *dev, union mlx90392_osr_dig_filt *val)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0x14, val, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Get OSR_DIG_FILT error\r\n");
    }

    return res;
}

rt_err_t mlx90392_set_osr_dig_filt(struct mlx90392_device *dev, union mlx90392_osr_dig_filt val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x14;
    send_buf[1] = val.byte_val;
    res = mlx90392_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("Set OSR_DIG_FILT error\r\n");
    }

    return res;
}

rt_err_t mlx90392_get_cust_ctrl(struct mlx90392_device *dev, union mlx90392_cust_ctrl *val)
{
    rt_err_t res = RT_EOK;

    res = mlx90392_mem_read(dev, 0x15, val, 1);
    if (res != RT_EOK)
    {
        rt_kprintf("Get CUST_CTRL error\r\n");
    }

    return res;
}

rt_err_t mlx90392_set_cust_ctrl(struct mlx90392_device *dev, union mlx90392_cust_ctrl val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = 0x15;
    send_buf[1] = val.byte_val;
    res = mlx90392_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        rt_kprintf("Set CUST_CTRL error\r\n");
    }

    return res;
}

rt_err_t mlx90392_get_xyz(struct mlx90392_device *dev, struct mlx90392_xyz *xyz)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[6];

    res = mlx90392_mem_read(dev, 0x1, recv_buf, 6);
    if (res == RT_EOK)
    {
        xyz->x = recv_buf[1]<<8 | recv_buf[0];
        xyz->y = recv_buf[3]<<8 | recv_buf[2];
        xyz->z = recv_buf[5]<<8 | recv_buf[4];
    }

    return res;
}



static rt_err_t mlx90392_read_regs(struct mlx90392_device *dev, rt_uint8_t start_addr, rt_uint8_t *recv_buf, rt_uint8_t len)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t send_buf[2];

    send_buf[0] = start_addr;

    res = mlx90392_transfer(dev, send_buf, 1, recv_buf, len);
    if (res != RT_EOK)
    {
        rt_kprintf("error\r\n");
    }

    return res;
}

/**
 * This function writes the value of the register for mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90392
 * @param val value to write
 *
 * @return the writing status, RT_EOK represents writing the value of the register successfully.
 */
static rt_err_t mlx90392_write_reg(struct mlx90392_device *dev, rt_uint8_t reg, rt_uint16_t val)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t recv_buf[3];
    rt_uint8_t send_buf[] =
    {
        CMD_WRITE_REGISTER,
        (val&0xFF00) >> 8,
        val&0x00FF,
        reg << 2
    };

    res = mlx90392_transfer(dev, send_buf, 4, recv_buf, 1);

    return res;
}

static int count_set_bits(int N)
{
    int result = 0;

    while (N)
    {
        result++;
        N &=N-1;
    }

    return result;
}

rt_err_t mlx90392_read_measurement(struct mlx90392_device *dev, rt_int8_t zyxt, struct mlx90392_txyz *txyz)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t send_buf[2];
    rt_uint8_t recv_buf[10];

    send_buf[0] = (CMD_READ_MEASUREMENT)|(zyxt);
    for (int i=0; i<2*count_set_bits(zyxt); i++)
    {
        send_buf[i+2] = 0x00;
    }

    res = mlx90392_transfer(dev, send_buf, 1, recv_buf, 1+2*count_set_bits(zyxt));
    if (res == RT_EOK)
    {
        int idx = 1;
        if (zyxt & 0x1)
        {
            txyz->t = ((rt_uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }

        if (zyxt & 0x2)
        {
            txyz->x = ((rt_uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }

        if (zyxt & 0x4)
        {
            txyz->y = ((rt_uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }

        if (zyxt & 0x8)
        {
            txyz->z = ((rt_uint16_t)recv_buf[idx]) << 8 | recv_buf[idx+1];
            idx = idx + 2;
        }
    }

    return res;
}

/*
    Temperature sensor resolution       = 45.2 LSB/C
    Temperature sensor output at 25C    = 46244 LSB(16u)
    MLX90392 datasheet page 12
 */
rt_int16_t mlx90392_convert_temperature(rt_uint16_t raw)
{
    float temperature;

    temperature = 25 + (raw - 46244)/45.2;
    rt_kprintf("t = %d.%d C\r\n", (int)temperature, ((int)(temperature*10))%10);
}

rt_err_t mlx90392_convert_measurement(struct mlx90392_device *dev, struct mlx90392_txyz txyz)
{
    mlx90392_resolution_t res_x = MLX90392_RES_18;
    mlx90392_resolution_t res_y = MLX90392_RES_18;
    mlx90392_resolution_t res_z = MLX90392_RES_18;

    mlx90392_gain_t gain = 3;

    float x, y, z;

    mlx90392_get_resolution(dev, &res_x, &res_y, &res_z);

    mlx90392_get_gain_sel(dev, &gain);

    if (res_x == MLX90392_RES_18)
    {
        txyz.x -= 0x8000;
        rt_kprintf("txyz.x - 0x8000 = 0x%x\r\n", txyz.x);
    }

    if (res_x == MLX90392_RES_19)
    {
        txyz.x -= 0x4000;
        rt_kprintf("txyz.x - 0x4000 = 0x%x\r\n", txyz.x);
    }

    if (res_y == MLX90392_RES_18)
    {
        txyz.y -= 0x8000;
        rt_kprintf("txyz.y - 0x8000 = 0x%x\r\n", txyz.y);
    }

    if (res_y == MLX90392_RES_19)
    {
        txyz.y -= 0x4000;
        rt_kprintf("txyz.x - 0x4000 = 0x%x\r\n", txyz.y);
    }

    if (res_z == MLX90392_RES_18)
    {
        txyz.z -= 0x8000;
        rt_kprintf("txyz.z - 0x8000 = 0x%x\r\n", txyz.z);
    }

    if (res_z == MLX90392_RES_19)
    {
        txyz.z -= 0x4000;
        rt_kprintf("txyz.z - 0x4000 = 0x%x\r\n", txyz.z);
    }

    x = (float)txyz.x * mlx90392_lsb_lookup[0][gain][res_x][0];
    y = (float)txyz.y * mlx90392_lsb_lookup[0][gain][res_y][0];
    z = (float)txyz.z * mlx90392_lsb_lookup[0][gain][res_z][1];

    // rt_kprintf("%.3f uT %.3f uT %.3f uT\r\n", x, y, z);
    // rt_kprintf("0x%xuT 0x%xuT 0x%xuT\r\n", x, y, z);
    rt_kprintf("%duT %duT %duT\r\n", (int)x, (int)y, (int)z);
}

rt_err_t mlx90392_set_hallconf(struct mlx90392_device *dev, rt_uint8_t hallconf)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register0 reg;

    res = mlx90392_read_reg(dev, 0, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.hallconf = hallconf;
    res = mlx90392_write_reg(dev, 0, reg.word_val);
    if (res == -RT_ERROR)
        return res;
        
    return res;
}

rt_err_t mlx90392_set_gain_sel(struct mlx90392_device *dev, mlx90392_gain_t gain)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register0 reg;

    res = mlx90392_read_reg(dev, 0, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.gain_sel = gain;
    
    res = mlx90392_write_reg(dev, 0, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90392_get_gain_sel(struct mlx90392_device *dev, mlx90392_gain_t *gain)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register0 reg;

    res = mlx90392_read_reg(dev, 0, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *gain = reg.gain_sel;
    
    rt_kprintf("gain = 0x%x\r\n", *gain);

    return res;
}

rt_uint8_t mlx90392_set_burst_sel(struct mlx90392_device *dev, rt_uint8_t burst_sel)
{
    rt_uint16_t register_val;
    union mlx90392_register1 reg;

    rt_uint8_t status1 = mlx90392_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.burst_sel = burst_sel;
    rt_uint8_t status2 = mlx90392_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

rt_uint8_t mlx90392_set_external_trigger(struct mlx90392_device *dev, rt_uint8_t ext_trg)
{
    rt_uint16_t register_val;
    union mlx90392_register1 reg;

    rt_uint8_t status1 = mlx90392_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.ext_trg = ext_trg;
    rt_uint8_t status2 = mlx90392_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

rt_uint8_t mlx90392_set_trigger_interrup_sel(struct mlx90392_device *dev, rt_uint8_t trig_int)
{
    rt_uint16_t register_val;
    union mlx90392_register1 reg;

    rt_uint8_t status1 = mlx90392_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.trig_int = trig_int;
    rt_uint8_t status2 = mlx90392_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

rt_uint8_t mlx90392_set_temperature_compensation(struct mlx90392_device *dev, rt_uint8_t on_off)
{
    rt_uint16_t register_val;
    union mlx90392_register1 reg;

    rt_uint8_t status1 = mlx90392_read_reg(dev, 1, &register_val);
    reg.word_val = register_val;
    reg.tcmp_en = on_off;
    rt_uint8_t status2 = mlx90392_write_reg(dev, 1, reg.word_val);

    return (status1) | (status2);
}

rt_err_t mlx90392_set_resolution(struct mlx90392_device *dev, mlx90392_resolution_t res_x, mlx90392_resolution_t res_y, mlx90392_resolution_t res_z)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register2 reg;

    res = mlx90392_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.res_x = res_x;
    reg.res_y = res_y;
    reg.res_z = res_z;
    
    res = mlx90392_write_reg(dev, 2, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90392_get_resolution(struct mlx90392_device *dev, mlx90392_resolution_t *res_x, mlx90392_resolution_t *res_y, mlx90392_resolution_t *res_z)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register2 reg;

    res = mlx90392_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *res_x = reg.res_x;
    *res_y = reg.res_y;
    *res_z = reg.res_z;
    
    rt_kprintf("res_x = 0x%x, res_y = 0x%x, res_z = 0x%x\r\n", *res_x, *res_y, *res_z);

    return res;
}

rt_err_t mlx90392_set_oversampling(struct mlx90392_device *dev, mlx90392_oversampling_t osr)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register2 reg;

    res = mlx90392_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.osr = osr;
    res = mlx90392_write_reg(dev, 2, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90392_get_oversampling(struct mlx90392_device *dev, mlx90392_oversampling_t *osr)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register2 reg;

    res = mlx90392_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *osr = reg.osr;
        
    return res;
}

rt_err_t mlx90392_set_digital_filtering(struct mlx90392_device *dev, mlx90392_filter_t dig_filt)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register2 reg;

    res = mlx90392_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    reg.dig_filt = dig_filt;
    res = mlx90392_write_reg(dev, 2, reg.word_val);
    if (res == -RT_ERROR)
        return res;

    return res;
}

rt_err_t mlx90392_get_digital_filtering(struct mlx90392_device *dev, mlx90392_filter_t *dig_filt)
{
    rt_err_t res = 0;

    rt_uint16_t register_val;
    union mlx90392_register2 reg;

    res = mlx90392_read_reg(dev, 2, &register_val);
    if (res == -RT_ERROR)
        return res;

    reg.word_val = register_val;
    *dig_filt = reg.dig_filt;

    return res;
}

rt_uint8_t mlx90392_set_offset_x(struct mlx90392_device *dev, rt_uint16_t offset)
{
    rt_uint8_t status = mlx90392_write_reg(dev, 4, offset);

    return status;
}

rt_uint8_t mlx90392_set_offset_y(struct mlx90392_device *dev, rt_uint16_t offset)
{
    rt_uint8_t status = mlx90392_write_reg(dev, 5, offset);

    return status;
}

rt_uint8_t mlx90392_set_offset_z(struct mlx90392_device *dev, rt_uint16_t offset)
{
    rt_uint8_t status = mlx90392_write_reg(dev, 6, offset);

    return status;
}

rt_uint8_t mlx90392_set_woxy_threshold(struct mlx90392_device *dev, rt_uint16_t woxy_threshold)
{
    rt_uint8_t status = mlx90392_write_reg(dev, 7, woxy_threshold);

    return status;
}

rt_uint8_t mlx90392_set_woz_threshold(struct mlx90392_device *dev, rt_uint16_t woz_threshold)
{
    rt_uint8_t status = mlx90392_write_reg(dev, 6, woz_threshold);

    return status;
}

rt_uint8_t mlx90392_set_wot_threshold(struct mlx90392_device *dev, rt_uint16_t wot_threshold)
{
    rt_uint8_t status = mlx90392_write_reg(dev, 6, wot_threshold);

    return status;
}

void mlx90392_setup(struct mlx90392_device *dev)
{
//    mlx90392_reset(dev);

//    rt_thread_delay(10000);

    mlx90392_set_gain_sel(dev, 4);
    mlx90392_set_resolution(dev, 0, 0, 0);
    mlx90392_set_oversampling(dev, 3);
    mlx90392_set_digital_filtering(dev, 7);
    mlx90392_set_temperature_compensation(dev, 0);
}

/**
 * This function gets the raw data of mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param txyz the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx90392_get_txyz_raw(struct mlx90392_device *dev, struct mlx90392_txyz *txyz)
{
    rt_uint8_t status = mlx90392_start_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);

    // wait for DRDY signal if connected, otherwise delay appropriately
//    if (DRDY_pin >= 0)
//    {
//      delayMicroseconds(600);
//      while (!digitalRead(DRDY_pin))
//      {
//        // busy wait
//      }
//    }
//    else
//    {
//      delay(this->convDelayMillis());
//    }

    status = mlx90392_read_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, txyz);
//    data = convertRaw(raw_txyz);

    return status;
}

/**
 * This function gets mlx90392 parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param read data pointer
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx90392_get_param(struct mlx90392_device *dev, enum mlx90392_cmd cmd, rt_uint16_t *param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    // switch (cmd)
    // {
    // case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, &data);
    //     *param = data;
    //     break;
    // case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, &data);
    //     *param = data;
    //     break;
    // case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
    //     *param = data;
    //     break;
    // case MPU6XXX_SAMPLE_RATE: /* Sample Rate */
    //     /* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
    //     if (res != RT_EOK)
    //     {
    //         break;
    //     }

    //     if (data == 0 || data == 7) /* dlpf is disable */
    //     {
    //         res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
    //         *param = 8000 / (data + 1);
    //     }
    //     else /* dlpf is enable */
    //     {
    //         res = mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
    //         *param = 1000 / (data + 1);
    //     }
    //     break;
    // case MPU6XXX_SLEEP: /* sleep mode */
    //     res = mpu6xxx_read_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, &data);
    //     *param = data;
    //     break;
    // }

    return res;
}

/**
 * This function set mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK represents  setting the parameter successfully.
 */
rt_err_t mlx90392_set_param(struct mlx90392_device *dev, enum mlx90392_cmd cmd, rt_uint16_t param)
{
    rt_uint8_t data = 0;
    rt_err_t res = RT_EOK;

    RT_ASSERT(dev);

    // switch (cmd)
    // {
    // case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
    //     res = mpu6xxx_write_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, param);
    //     dev->config.gyro_range = param;
    //     break;
    // case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
    //     res = mpu6xxx_write_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, param);
    //     dev->config.accel_range = param;
    //     break;
    // case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
    //     res = mpu6xxx_write_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, param);
    //     break;
    // case MPU6XXX_SAMPLE_RATE: /* Sample Rate = 16-bit unsigned value.
    //                              Sample Rate = [1000 -  4]HZ when dlpf is enable
    //                              Sample Rate = [8000 - 32]HZ when dlpf is disable */

    //     //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    //     res = mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
    //     if (res != RT_EOK)
    //     {
    //         break;
    //     }

    //     if (data == 0 || data == 7) /* dlpf is disable */
    //     {
    //         if (param > 8000)
    //             data = 0;
    //         else if (param < 32)
    //             data = 0xFF;
    //         else
    //             data = 8000 / param - 1;
    //     }
    //     else /* dlpf is enable */
    //     {
    //         if (param > 1000)
    //             data = 0;
    //         else if (param < 4)
    //             data = 0xFF;
    //         else
    //             data = 1000 / param - 1;
    //     }
    //     res = mpu6xxx_write_reg(dev, MPU6XXX_RA_SMPLRT_DIV, data);
    //     break;
    // case MPU6XXX_SLEEP: /* Configure sleep mode */
    //     res = mpu6xxx_write_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, param);
    //     break;
    // }

    return res;
}

/**
 * This function initialize the mlx90392 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL represents  initialization failed.
 */
struct mlx90392_device *mlx90392_init(const char *dev_name, rt_uint8_t param)
{
    struct mlx90392_device *dev = RT_NULL;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mlx90392_device));
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't allocate memory for mlx90392 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        rt_kprintf("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            rt_uint8_t id[2];

            /* find mlx90392 device at address: 0x0C */
            dev->i2c_addr = MLX90392_I2C_ADDRESS;
            if (mlx90392_mem_read(dev, 0x0A, id, 2) != RT_EOK)
            {
                rt_kprintf("Can't find device at '%s'!", dev_name);
                goto __exit;
            }
            else
            {
                rt_kprintf("CID is 0x%x\r\n", id[0]);
                rt_kprintf("DID is 0x%x\r\n", id[1]);
            }

            rt_kprintf("Device i2c address is:'0x%x'!\r\n", dev->i2c_addr);
        }
#endif        
    }
    else
    {
        rt_kprintf("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }
    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90392_deinit(struct mlx90392_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mlx90392(int argc, char **argv)
{
    rt_uint16_t register_val;
    static struct mlx90392_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mlx90392 [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mlx90392 by given name, ex:i2c2\n");
        rt_kprintf("         rr <reg>              Set sample rate to var\n");
        rt_kprintf("                               var = [1000 -  4] when dlpf is enable\n");
        rt_kprintf("                               var = [8000 - 32] when dlpf is disable\n");
        rt_kprintf("         wr <reg> <var>        Set gyro range to var\n");
        rt_kprintf("                               var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("         ar <var>              Set accel range to var\n");
        rt_kprintf("                               var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("         sleep <var>           Set sleep status\n");
        rt_kprintf("                               var = 0 means disable, = 1 means enable\n");
        rt_kprintf("         read [num]            read [num] times mlx90392\n");
        rt_kprintf("                               num default 5\n");
        return;
    }
    else
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mlx90392_deinit(dev);
            }

            if (argc == 2)
                dev = mlx90392_init("i2c2", RT_NULL);
            else if (argc == 3)
                dev = mlx90392_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            rt_kprintf("Please probe mlx90392 first!\n");
            return;
        }
        else if (!strcmp(argv[1], "rt"))
        {
            mlx90392_reset(dev);
        }
        else if (!strcmp(argv[1], "rrs"))
        {
            rt_uint8_t regs[0x16];
            rt_uint8_t start_addr = atoi(argv[2]);
            rt_uint8_t len = atoi(argv[3]);

            mlx90392_read_regs(dev, start_addr, regs, len);

            for (int i=0; i<len; i++)
                rt_kprintf("Reading REG[%d] = 0x%x\r\n", start_addr+i, regs[i]);
        }
        else if (!strcmp(argv[1], "id"))
        {
            rt_uint8_t id[2];
            rt_uint8_t start_addr = 10;
            rt_uint8_t len = 2;

            mlx90392_mem_read(dev, start_addr, id, len);
            rt_kprintf("CID = 0x%x\r\n", id[0]);
            rt_kprintf("DID = 0x%x\r\n", id[1]);
        }
        else if (!strcmp(argv[1], "stat1"))
        {
            union mlx90392_stat1 stat1;
            mlx90392_get_stat1(dev, &stat1);
        }
        else if (!strcmp(argv[1], "x"))
        {
            rt_int16_t x;

            mlx90392_get_x(dev, &x);
            rt_kprintf("x = 0x%x\r\n", x);
        }
        else if (!strcmp(argv[1], "y"))
        {
            rt_int16_t y;

            mlx90392_get_y(dev, &y);
            rt_kprintf("y = 0x%x\r\n", y);
        }
        else if (!strcmp(argv[1], "z"))
        {
            rt_int16_t z;

            mlx90392_get_z(dev, &z);
            rt_kprintf("z = 0x%x\r\n", z);
        }
        else if (!strcmp(argv[1], "rr"))
        {
            union mlx90392_register0 reg0;
            union mlx90392_register1 reg1;
            union mlx90392_register2 reg2;
            union mlx90392_register3 reg3;

            mlx90392_read_reg(dev, atoi(argv[2]), &register_val);
            rt_kprintf("Reading REG[%d] = 0x%x...\r\n", atoi(argv[2]), register_val);

            switch (atoi(argv[2]))
            {
            case 0:
                reg0.word_val = register_val;
                rt_kprintf("REG[0] = 0x%x\r\n", reg0.word_val);
                rt_kprintf("[BIT0-3] HALLCONF = 0x%x - Hall plate spinning rate adjustment\r\n", reg0.hallconf);
                rt_kprintf("[BIT4-6] GAIN_SEL = 0x%x - Analog chain gain setting, factor 5 between min and max code\r\n", reg0.gain_sel);
                rt_kprintf("[BIT7-7] Z_SERIES = 0x%x - Enable all plates for Z-measurement\r\n", reg0.z_series);
                rt_kprintf("[BIT8-8] BITS     = 0x%x - Enable the on-chip coil, applying a Z-field[Built-in Self Test]\r\n", reg0.bist);
                rt_kprintf("[BIT9-F] ANA_RESERVED_LOW = 0x%x - Reserved IO trimming bits\r\n", reg0.ana_reserved_low);
                break;
            default:
                rt_kprintf("REG[%d] = 0x%x\r\n", atoi(argv[2]), register_val);
                break;
            }
        }
        else if (!strcmp(argv[1], "wr"))
        {
            mlx90392_write_reg(dev, atoi(argv[2]), atoi(argv[3]));
        }
        else if (!strcmp(argv[1], "sm"))
        {
            mlx90392_start_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);
        }        
        else if (!strcmp(argv[1], "rm"))
        {
            struct mlx90392_txyz txyz;

            mlx90392_start_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);
            
            // rt_thread_delay(mlx90392_tconv[_dig_filt][_osr] + 10);
            rt_thread_delay(mlx90392_tconv[0][0] + 10);

            mlx90392_read_measurement(dev, X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, &txyz);

            mlx90392_convert_temperature(txyz.t);
            mlx90392_convert_measurement(dev, txyz);
        }                
        else if (!strcmp(argv[1], "set_gain"))
        {
            mlx90392_set_gain_sel(dev, atoi(argv[2]));
        }                
        else if (!strcmp(argv[1], "get_gain"))
        {
            mlx90392_gain_t gain;

            mlx90392_get_gain_sel(dev, &gain);
            rt_kprintf("gain is 0x%x\r\n", gain);
        }                                
        else if (!strcmp(argv[1], "set_resolution"))
        {
            mlx90392_set_resolution(dev, atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        }                
        else if (!strcmp(argv[1], "get_resolution"))
        {
            mlx90392_resolution_t x;
            mlx90392_resolution_t y;
            mlx90392_resolution_t z;

            mlx90392_get_resolution(dev, &x, &y, &z);
            rt_kprintf("resolution is 0x%x 0x%x 0x%x\r\n", x, y, z);
        }                                        
        else if (!strcmp(argv[1], "setup"))
        {
            mlx90392_setup(dev);
        }
        else if (!strcmp(argv[1], "readdata"))
        {
            struct mlx90392_txyz txyz;
            mlx90392_get_txyz_raw(dev, &txyz);
        }
        else
        {
            rt_kprintf("Unknown command, please enter 'mlx90392' get help information!\n");
        }
    }
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(mlx90392, mlx90392 sensor function);
#endif

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90392, mlx90392 sensor function);
#endif
