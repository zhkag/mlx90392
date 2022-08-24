/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __MLX90392_H__
#define __MLX90392_H__

#include <rtthread.h>

enum pn
{
    MLX90392ELQ_AAA_010,
    MLX90392ELQ_AAA_011,
    MLX90392ELQ_AAA_013,
};

#define MLX90392    MLX90392ELQ_AAA_011

#if MLX90392 == MLX90392ELQ_AAA_010
#define MLX90392_I2C_ADDRESS                    (0x0C)        // address pin A0,A1 low (GND), default for MLX90392
#elif MLX90392 == MLX90392ELQ_AAA_011
#define MLX90392_I2C_ADDRESS                    (0x0C)        // address pin A0,A1 low (GND), default for MLX90392
#elif MLX90392 == MLX90392ELQ_AAA_013
#define MLX90392_I2C_ADDRESS                    (0x3C)        // address pin A0,A1 low (GND), default for MLX90392
#endif

union mlx90392_stat1
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t drdy     : 1;    //BIT0
        rt_uint8_t stat1_1  : 1;
        rt_uint8_t stat1_2  : 1;
        rt_uint8_t rt       : 1;
        rt_uint8_t stat1_4  : 1;
        rt_uint8_t stat1_5  : 1;
        rt_uint8_t stat1_6  : 1;
        rt_uint8_t stat1_7  : 1;
    };
};

union mlx90392_stat2
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t hovf     : 1;    //BIT0
        rt_uint8_t dor      : 1;
        rt_uint8_t stat2_2  : 1;
        rt_uint8_t stat2_3  : 1;
        rt_uint8_t stat2_4  : 1;
        rt_uint8_t stat2_5  : 1;
        rt_uint8_t stat2_6  : 1;
        rt_uint8_t stat2_7  : 1;
    };
};

/* 3-axis data structure */
struct mlx90392_xyz
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

enum mlx90392_mode
{
    POWER_DOWN_MODE                     = 0x0,
    SINGLE_MEASUREMENT_MODE             = 0x1,
    CONTINUOUS_MEASUREMENT_MODE_10HZ    = 0x2,
    CONTINUOUS_MEASUREMENT_MODE_20HZ    = 0x3,
    CONTINUOUS_MEASUREMENT_MODE_50HZ    = 0x4,
    CONTINUOUS_MEASUREMENT_MODE_100HZ   = 0x5,
//    SELF_TEST_MODE                      = 0x6,
//    POWER_DOWN_MODE                     = 0x7,
//    POWER_DOWN_MODE                     = 0x8,
//    SINGLE_MEASUREMENT_MODE             = 0x9,
    CONTINUOUS_MEASUREMENT_MODE_200HZ   = 0xA,
    CONTINUOUS_MEASUREMENT_MODE_500HZ   = 0xB,
    CONTINUOUS_MEASUREMENT_MODE_700HZ   = 0xC,
    CONTINUOUS_MEASUREMENT_MODE_1400HZ  = 0xD,
//    SELF_TEST_MODE                      = 0xE,
//    POWER_DOWN_MODE                     = 0xF
};

union mlx90392_osr_dig_filt
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t dig_filt_temp     : 3;
        rt_uint8_t dig_filt_hall_xy  : 3;
        rt_uint8_t osr_temp          : 1;
        rt_uint8_t osr_hall          : 1;    //BIT7
    };
};

union mlx90392_cust_ctrl
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t dig_filt_hall_z  : 3;
        rt_uint8_t cust_ctrl3       : 1;
        rt_uint8_t dnc3_1           : 1;
        rt_uint8_t t_comp_en        : 1;
        rt_uint8_t dnc2_0           : 1;
        rt_uint8_t dnc1_1           : 1;    //BIT7
    };
};


enum cmd
{
    CMD_NOP               = 0x00,
    CMD_EXIT              = 0x80,
    CMD_START_BURST       = 0x10,
    CMD_WAKE_ON_CHANGE    = 0x20,
    CMD_START_MEASUREMENT = 0x30,
    CMD_READ_MEASUREMENT  = 0x40,
    CMD_READ_REGISTER     = 0x50,
    CMD_WRITE_REGISTER    = 0x60,
    CMD_MEMORY_RECALL     = 0xd0,
    CMD_MEMORY_STORE      = 0xe0,
    CMD_RESET             = 0xf0
};

enum
{
    Z_FLAG = 0x8,
    Y_FLAG = 0x4,
    X_FLAG = 0x2,
    T_FLAG = 0x1
} axis_flag_t;

/* Accelerometer full scale range */
enum mlx90392_accel_range
{
    MPU6XXX_ACCEL_RANGE_2G  = 0, // ±2G
    MPU6XXX_ACCEL_RANGE_4G  = 1, // ±4G
    MPU6XXX_ACCEL_RANGE_8G  = 2, // ±8G
    MPU6XXX_ACCEL_RANGE_16G = 3  // ±16G
};

/* sleep mode parameters */
enum mlx90392_sleep
{
    MPU6XXX_SLEEP_DISABLE = 0,
    MPU6XXX_SLEEP_ENABLE  = 1
};

/* Supported configuration items */
enum mlx90392_cmd
{
    MPU6XXX_GYRO_RANGE,  /* Gyroscope full scale range */
    MPU6XXX_ACCEL_RANGE, /* Accelerometer full scale range */
    MPU6XXX_DLPF_CONFIG, /* Digital Low Pass Filter */
    MPU6XXX_SAMPLE_RATE, /* Sample Rate —— 16-bit unsigned value.
                            Sample Rate = [1000 -  4]HZ when dlpf is enable
                            Sample Rate = [8000 - 32]HZ when dlpf is disable */
    MPU6XXX_SLEEP        /* Sleep mode */
};

/** HALLCONF settings for CONF1 register. */
typedef enum mlx90392_hallconf
{
    MLX90392_HALLCONF_0 = (0x0),
    MLX90392_HALLCONF_C = (0xC),
} mlx90392_hallconf_t;

/** Gain settings for CONF1 register. */
typedef enum mlx90392_gain
{
    MLX90392_GAIN_5X = (0x00),
    MLX90392_GAIN_4X,
    MLX90392_GAIN_3X,
    MLX90392_GAIN_2_5X,
    MLX90392_GAIN_2X,
    MLX90392_GAIN_1_67X,
    MLX90392_GAIN_1_33X,
    MLX90392_GAIN_1X
} mlx90392_gain_t;

/** Resolution settings for CONF3 register. */
typedef enum mlx90392_resolution
{
    MLX90392_RES_16,
    MLX90392_RES_17,
    MLX90392_RES_18,
    MLX90392_RES_19,
} mlx90392_resolution_t;

/** Digital filter settings for CONF3 register. */
typedef enum mlx90392_filter
{
    MLX90392_FILTER_0,
    MLX90392_FILTER_1,
    MLX90392_FILTER_2,
    MLX90392_FILTER_3,
    MLX90392_FILTER_4,
    MLX90392_FILTER_5,
    MLX90392_FILTER_6,
    MLX90392_FILTER_7,
} mlx90392_filter_t;

/** Oversampling settings for CONF3 register. */
typedef enum mlx90392_oversampling
{
    MLX90392_OSR_0,
    MLX90392_OSR_1,
    MLX90392_OSR_2,
    MLX90392_OSR_3,
} mlx90392_oversampling_t;

union mlx90392_status
{
    struct
    {
    /** D1-D0: indicates the number of bytes (2D[1:0]) to follow the status byte after a read measurement
     * or a read register command has been sent.
     */
    rt_uint8_t d0 : 1;
    /** D1-D0: indicates the number of bytes (2D[1:0]) to follow the status byte after a read measurement
     * or a read register command has been sent.
     */
    rt_uint8_t d1 : 1;

    /** RS: indicates that the device has been reset successfully by a reset command.
     */
    rt_uint8_t rs : 1;

    /** SED: indicates that a single bit error has been corrected by the NVRAM
     */
    rt_uint8_t sed : 1;

    /** ERROR: indicates an error.
     * Can be set when reading out a measurement while the measurement is not yet completed or
     * when reading out the same measurement twice.
     */
    rt_uint8_t error : 1;

    /** SM_mode: if set, the IC is executing a measurement sequence in polling mode.
     * It can be initiated by a SM command or a pulse on the TRIG input.
     */
    rt_uint8_t sm_mode : 1;

    /** WOC_mode: if set, the IC is in wake-up-on-change mode.
     */
    rt_uint8_t woc_mode : 1;

    /** Burst_mode: if set, the IC is working in burst mode.
     */
    rt_uint8_t burst_mode : 1;
    };

    rt_uint8_t byte_val;
};

/* mlx90392 config structure */
struct mlx90392_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* mlx90392 device structure */
struct mlx90392_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct mlx90392_config config;
};

/**
 * This function initialize the mlx90392 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mlx90392_device *mlx90392_init(const char *dev_name, rt_uint8_t param);

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90392_deinit(struct mlx90392_device *dev);

/**
 * This function set mlx90392 parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK reprensents  setting the parameter successfully.
 */
rt_err_t mlx90392_set_param(struct mlx90392_device *dev, enum mlx90392_cmd cmd, rt_uint16_t param);

/**
* This function gets the data of the mps, unit: mg
 *
 * @param dev the pointer of device driver structure
 * @param mps the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx90392_get_mps(struct mlx90392_device *dev, struct mlx90392_3axes *accel);

/**
* This function gets the data of the gyroscope, unit: deg/10s
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx90392_get_gyro(struct mlx90392_device *dev, struct mlx90392_3axes *gyro);

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mlx90392_get_temp(struct mlx90392_device *dev, float *temp);

/**
* This function sets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents setting the offsets successfully.
 */
rt_err_t mlx90392_set_accel_offset(struct mlx90392_device *dev, struct mlx90392_3axes *offset);

/**
* This function gets the offset of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents reading the offsets successfully.
 */
rt_err_t mlx90392_get_accel_offset(struct mlx90392_device *dev, struct mlx90392_3axes *offset);

/**
* This function sets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents setting the offsets successfully.
 */
rt_err_t mlx90392_set_gyro_offset(struct mlx90392_device *dev, struct mlx90392_3axes *offset);

/**
* This function gets the offset of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param offset the pointer of 3axes structure of offsets
 *
 * @return the setting status, RT_EOK reprensents reading the offsets successfully.
 */
rt_err_t mlx90392_get_gyro_offset(struct mlx90392_device *dev, struct mlx90392_3axes *offset);

rt_err_t mlx90392_nop(struct mlx90392_device *dev);
rt_err_t mlx90392_exit(struct mlx90392_device *dev);
rt_err_t mlx90392_reset(struct mlx90392_device *dev);

rt_err_t mlx90392_get_gain_sel(struct mlx90392_device *dev, mlx90392_gain_t *gain);
rt_err_t mlx90392_get_resolution(struct mlx90392_device *dev, mlx90392_resolution_t *res_x, mlx90392_resolution_t *res_y, mlx90392_resolution_t *res_z);

#endif
