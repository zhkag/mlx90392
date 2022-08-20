# API 说明

在RT-Thread上编程，需要用到迈来芯MLX90393三轴磁位置传感器时，使用mlx90393软件包就可以轻松完成传感器的配置以及传感器数据的读取，本章介绍mlx90393软件包提供的常用API。

### 初始化函数

```{.c}
struct mlx90393_device *mlx90393_init(const char *dev_name, rt_uint8_t param);
```

使用指定的通信设备（I2C/SPI）初始化mlx90393 ，并返回控制句柄。

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev_name               | 用于同 mlx90393 通信的设备名（支持 I2C 总线设备和 SPI 设备） |
|param | I2C 通信，根据此处传入的 I2C 地址寻找设备（例如：0x68） |
| **返回**          | **描述**                                |
|struct mlx90393_device *                  | mlx90393_device 结构体的指针，它在调用 mlx90393 库的其他函数时使用 |
|NULL                 | 失败                                |

### 反初始化函数

```{.c}
void mlx90393_deinit(struct mlx90393_device *dev);
```

释放 mlx90393 设备占据的内存空间

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mlx90393_device 结构体的指针 |
| **返回** | **描述**                    |
| 无返回值 |                             |

### 设定参数

```{.c}
rt_err_t mlx90393_set_param(struct mlx90393_device *dev, enum mlx90393_cmd cmd, rt_uint16_t param);
```

为挂载上的 mlx90393 设备设定参数

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|dev               | mlx90393_device 结构体的指针 |
|cmd | 支持的配置选项，详见下面的介绍 |
|param | 设定的具体的参数值 |
| **返回**          | **描述**                                |
|RT_EOK                  | 成功 |
|< 0                 | 失败                                |

cmd 参数指要配置的选项，param 参数表示设定的参数的具体值。详情如下:

**支持的配置选项** 

| 参数              | 描述                                |
|:------------------|:------------------------------------|
|mlx90393_GYRO_RANGE               | 配置陀螺仪的测量范围 |
|mlx90393_ACCEL_RANGE | 配置加速度计的测量范围 |
|mlx90393_DLPF_CONFIG | 配置低通滤波器 |
| mlx90393_SAMPLE_RATE | 配置采样率 |
|mlx90393_SLEEP                  | 配置休眠模式 |

**mlx90393_GYRO_RANGE 参数的值** 

```
mlx90393_GYRO_RANGE_250DPS  // ±250°/s
mlx90393_GYRO_RANGE_500DPS  // ±500°/s
mlx90393_GYRO_RANGE_1000DPS // ±1000°/s
mlx90393_GYRO_RANGE_2000DPS // ±2000°/s
```

**mlx90393_ACCEL_RANGE 参数的值** 

```
mlx90393_ACCEL_RANGE_2G   // ±2G
mlx90393_ACCEL_RANGE_4G   // ±4G
mlx90393_ACCEL_RANGE_8G   // ±8G
mlx90393_ACCEL_RANGE_16G  // ±16G
```

**mlx90393_DLPF_CONFIG 参数的值** 

```
mlx90393_DLPF_DISABLE  //256HZ
mlx90393_DLPF_188HZ    //188HZ
mlx90393_DLPF_98HZ
mlx90393_DLPF_42HZ
mlx90393_DLPF_20HZ
mlx90393_DLPF_10HZ
mlx90393_DLPF_5HZ
```

**mlx90393_SAMPLE_RATE 参数的值** 

```
传入采样率的值（16-bit unsigned value.）
如果使能了低通滤波器（dlpf）的话，范围为 1000HZ - 4HZ。否则，范围为 8000HZ - 32HZ
```

**mlx90393_SLEEP  参数的值** 

```
mlx90393_SLEEP_DISABLE //退出休眠模式
mlx90393_SLEEP_ENABLE  //进入休眠模式
```


### 读取陀螺仪数据   

```{.c}
rt_err_t mlx90393_get_gyro(struct mlx90393_device *dev, struct mlx90393_3axes *gyro);
```

读取陀螺仪数据 （单位： deg/10s）

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | mlx90393_device 结构体的指针             |
| gyro     | 存储 mlx90393 3轴陀螺仪数据 结构体的指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| < 0      | 失败                                    |

3 轴陀螺仪数据的结构体定义如下

```{.c}
struct mlx90393_3axes
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};
```

### 读取加速度计数据

```{.c}
rt_err_t mlx90393_get_accel(struct mlx90393_device *dev, struct mlx90393_3axes *accel);
```

读取加速度计数据 （单位： mg）

| 参数     | 描述                                    |
| :------- | :-------------------------------------- |
| dev      | mlx90393_device 结构体的指针             |
| accel    | 存储 mlx90393 3轴加速度数据 结构体的指针 |
| **返回** | **描述**                                |
| RT_EOK   | 成功                                    |
| < 0      | 失败                                    |

## 校准传感器

### 校准陀螺仪

```c
rt_err_t mlx90393_set_gyro_offset(struct mlx90393_device *dev, struct mlx90393_3axes *offset);
```

 校准陀螺仪。注意offset的单位和读取数据的单位不同。

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mlx90393_device 结构体的指针 |
| offset   | 存储 校准量 结构体的指针    |
| **返回** | **描述**                    |
| RT_EOK   | 成功                        |
| 其他     | 失败                        |

### 校准加速度传感器

```c
rt_err_t mlx90393_set_gyro_offset(struct mlx90393_device *dev, struct mlx90393_3axes *offset);
```

校准加速度传感器。注意offset的单位和读取数据的单位不同。

| 参数     | 描述                        |
| :------- | :-------------------------- |
| dev      | mlx90393_device 结构体的指针 |
| offset   | 存储 校准量 结构体的指针    |
| **返回** | **描述**                    |
| RT_EOK   | 成功                        |
| 其他     | 失败                        |

### 读取温度计数据

```{.c}
rt_err_t mlx90393_get_temp(struct mlx90393_device *dev, float *temp);
```

读取温度计数据 （单位：摄氏度）

| 参数     | 描述                            |
| :------- | :------------------------------ |
| dev      | mlx90393_device 结构体的指针     |
| temp     | 存储 mlx90393 温度数据地址的指针 |
| **返回** | **描述**                        |
| RT_EOK   | 成功                            |
| < 0      | 失败                            |
