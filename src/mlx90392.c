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
 * This function reads the value of register for mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90392
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
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

/**
 * This function reads the value of register for mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90392
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
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

/**
 * This function reads the value of register for mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90392
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
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
        rt_kprintf("STAT1 = 0x%x, DRDY = 0x%x\r\n", stat1->byte_val, stat1->drdy);
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

rt_err_t mlx90392_get_x_flux(struct mlx90392_device *dev, float *x)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x1, recv_buf, 2);
    if (res == RT_EOK)
    {
        *x = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0])*MAGNETO10_MAG_FLUX_RESOLUTION;
    }

    return res;
}

rt_err_t mlx90392_get_y_flux(struct mlx90392_device *dev, float *y)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x3, recv_buf, 2);
    if (res == RT_EOK)
    {
        *y = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0])*MAGNETO10_MAG_FLUX_RESOLUTION;
    }

    return res;
}

rt_err_t mlx90392_get_z_flux(struct mlx90392_device *dev, float *z)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x5, recv_buf, 2);
    if (res == RT_EOK)
    {
        *z = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0])*MAGNETO10_MAG_FLUX_RESOLUTION;
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

rt_err_t mlx90392_get_temperature(struct mlx90392_device *dev, float *t)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    res = mlx90392_mem_read(dev, 0x8, recv_buf, 2);
    if (res == RT_EOK)
    {
        *t = (float)(((rt_int16_t)recv_buf[1] << 8 ) | recv_buf[0] ) / MAGNETO10_TEMPERATURE_RES;
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

    res = mlx90392_mem_read(dev, 0x14, (rt_uint8_t *)val, 1);
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

    res = mlx90392_mem_read(dev, 0x15, (rt_uint8_t *)val, 1);
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

rt_err_t mlx90392_set_temperature(struct mlx90392_device *dev, rt_uint8_t onoff)
{
    rt_err_t res = RT_EOK;
    union mlx90392_cust_ctrl val;

    res = mlx90392_get_cust_ctrl(dev, &val);

    if (1 == onoff)
    {
        if (val.t_comp_en == 0)
        {
            val.t_comp_en = 1;
            res = mlx90392_set_cust_ctrl(dev, val);
        }
    }
    else
    {
        if (val.t_comp_en == 1)
        {
            val.t_comp_en = 0;
            res = mlx90392_set_cust_ctrl(dev, val);
        }
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

rt_err_t mlx90392_get_xyz_flux(struct mlx90392_device *dev, struct mlx90392_xyz_flux *xyz)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[6];

    res = mlx90392_mem_read(dev, 0x1, recv_buf, 6);
    if (res == RT_EOK)
    {
        xyz->x = (float)(((rt_int16_t)recv_buf[1] << 8) | recv_buf[0]) * MAGNETO10_MAG_FLUX_RESOLUTION;
        xyz->y = (float)(((rt_int16_t)recv_buf[3] << 8) | recv_buf[2]) * MAGNETO10_MAG_FLUX_RESOLUTION;
        xyz->z = (float)(((rt_int16_t)recv_buf[5] << 8) | recv_buf[4]) * MAGNETO10_MAG_FLUX_RESOLUTION;
    }

    return res;
}



rt_err_t mlx90392_set_hallconf(struct mlx90392_device *dev, rt_uint8_t hallconf)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx90392_register0 reg;
//
//    res = mlx90392_read_reg(dev, 0, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    reg.hallconf = hallconf;
//    res = mlx90392_write_reg(dev, 0, reg.word_val);
//    if (res == -RT_ERROR)
//        return res;
        
    return res;
}


rt_err_t mlx90392_set_oversampling(struct mlx90392_device *dev, mlx90392_oversampling_t osr)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx90392_register2 reg;
//
//    res = mlx90392_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    reg.osr = osr;
//    res = mlx90392_write_reg(dev, 2, reg.word_val);
//    if (res == -RT_ERROR)
//        return res;

    return res;
}

rt_err_t mlx90392_get_oversampling(struct mlx90392_device *dev, mlx90392_oversampling_t *osr)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx90392_register2 reg;
//
//    res = mlx90392_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    *osr = reg.osr;
        
    return res;
}

rt_err_t mlx90392_set_digital_filtering(struct mlx90392_device *dev, mlx90392_filter_t dig_filt)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx90392_register2 reg;
//
//    res = mlx90392_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    reg.dig_filt = dig_filt;
//    res = mlx90392_write_reg(dev, 2, reg.word_val);
//    if (res == -RT_ERROR)
//        return res;

    return res;
}

rt_err_t mlx90392_get_digital_filtering(struct mlx90392_device *dev, mlx90392_filter_t *dig_filt)
{
    rt_err_t res = 0;

//    rt_uint16_t register_val;
//    union mlx90392_register2 reg;
//
//    res = mlx90392_read_reg(dev, 2, &register_val);
//    if (res == -RT_ERROR)
//        return res;
//
//    reg.word_val = register_val;
//    *dig_filt = reg.dig_filt;

    return res;
}

void mlx90392_setup(struct mlx90392_device *dev)
{
//    mlx90392_reset(dev);

//    rt_thread_delay(10000);

//    mlx90392_set_gain_sel(dev, 4);
//    mlx90392_set_resolution(dev, 0, 0, 0);
//    mlx90392_set_oversampling(dev, 3);
//    mlx90392_set_digital_filtering(dev, 7);
//    mlx90392_set_temperature_compensation(dev, 0);
}

/**
 * This function gets the raw data of mlx90392
 *
 * @param dev the pointer of device driver structure
 * @param xyz the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx90392_continuous_measurement(struct mlx90392_device *dev, struct mlx90392_xyz *xyz, rt_uint8_t freq)
{
    rt_uint8_t status = RT_EOK;
    union mlx90392_stat1 stat1;

    switch (freq)
    {
    case 10:
        rt_kprintf("10Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_10HZ);
        break;
    case 20:
        rt_kprintf("20Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_20HZ);
        break;
    case 50:
        rt_kprintf("50Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_50HZ);
        break;
    case 100:
        rt_kprintf("100Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_100HZ);
        break;
    case 200:
        rt_kprintf("200Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_200HZ);
        break;
    case 500:
        rt_kprintf("500Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_500HZ);
        break;
    case 700:
        rt_kprintf("700Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_700HZ);
        break;
    case 1400:
        rt_kprintf("1400Hz");
        status = mlx90392_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_1400HZ);
        break;
    default:
        rt_kprintf("wrong freqency\r\n");
        break;
    }

    while (1)
    {
        status = mlx90392_get_stat1(dev, &stat1);

        if (stat1.drdy == 1)
        {
            status = mlx90392_get_xyz(dev, xyz);
            rt_kprintf("x = 0x%x, y = 0x%x, z = 0x%x\r\n", xyz->x, xyz->y, xyz->z);
        }

        rt_thread_delay(100);
    }

    return status;
}

static rt_err_t mlx90392_single_measurement(struct mlx90392_device *dev, struct mlx90392_xyz_flux *xyz)
{
    rt_uint8_t status = RT_EOK;
    union mlx90392_stat1 stat1;

    status = mlx90392_set_mode(dev, SINGLE_MEASUREMENT_MODE);

    stat1.byte_val = 0;
    while (stat1.drdy == 0)
    {
        status = mlx90392_get_stat1(dev, &stat1);
        rt_thread_delay(100);
    }

    status = mlx90392_get_xyz_flux(dev, xyz);

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
    static struct mlx90392_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mlx90392 [OPTION] [PARAM]\n");
        rt_kprintf("         probe <dev_name>      Probe mlx90392 by given name, ex:i2c2\n");
        rt_kprintf("         id                    Print CID and DID\n");
        rt_kprintf("         stat1                 Print stat1\n");
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
        else if (!strcmp(argv[1], "t"))
        {
            float t;

            mlx90392_get_temperature(dev, &t);
            rt_kprintf("t = %d.%d\r\n", (rt_int16_t)t, (rt_uint16_t)t*10%10);
        }
        else if (!strcmp(argv[1], "rr"))
        {
            rt_uint8_t val;
            mlx90392_mem_read(dev, atoi(argv[2]), &val, 1);

            rt_kprintf("Reading REG[%d] = 0x%x...\r\n", atoi(argv[2]), val);
        }
        else if (!strcmp(argv[1], "setup"))
        {
            mlx90392_setup(dev);
        }
        else if (!strcmp(argv[1], "xyz"))
        {
            struct mlx90392_xyz_flux xyz;

            mlx90392_single_measurement(dev, &xyz);
            rt_kprintf("x = %d.%d\r\n", (rt_int16_t)xyz.x, (rt_int16_t)xyz.x*10%10);
            rt_kprintf("y = %d.%d\r\n", (rt_int16_t)xyz.y, (rt_int16_t)xyz.y*10%10);
            rt_kprintf("z = %d.%d\r\n", (rt_int16_t)xyz.z, (rt_int16_t)xyz.z*10%10);
        }
        else if (!strcmp(argv[1], "continuous"))
        {
            struct mlx90392_xyz xyz;

            mlx90392_continuous_measurement(dev, &xyz, atoi(argv[2]));
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
