/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include <rtthread.h>
#include "mlx90393.h"

/* Default configuration, please change according to the actual situation, support i2c and spi device name */
#define MLX90393_DEVICE_NAME  "i2c2"

/* Test function */
static int mlx90393_test()
{
    struct mlx90393_device *dev;
    struct mpu6xxx_3axes accel, gyro;
    int i;

    /* Initialize mlx90393, The parameter is RT_NULL, means auto probing for i2c*/
    dev = mlx90393_init(MLX90393_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mlx90393 init failed\n");
        return -1;
    }
    rt_kprintf("mlx90393 init succeed\n");

    for (i = 0; i < 5; i++)
    {
        // mpu6xxx_get_accel(dev, &accel);
        // mpu6xxx_get_gyro(dev, &gyro);

        // rt_kprintf("accel.x = %3d, accel.y = %3d, accel.z = %3d ", accel.x, accel.y, accel.z);
        // rt_kprintf("gyro.x = %3d gyro.y = %3d, gyro.z = %3d\n", gyro.x, gyro.y, gyro.z);

        rt_thread_mdelay(100);
    }

    mlx90393_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mlx90393_test, mlx90393 sensor test function);
